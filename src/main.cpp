/*
 * SenseCAP Indicator D1 — Motion Camera Viewer (PlatformIO port of v7)
 *
 * Earlier theory (dead R4/GPIO0, B3/B4 leaking into green) was WRONG — a
 * misdiagnosis of a pixel clock/init-sequence/CS-framing bug, not damaged
 * hardware. Proven by: (1) openHASP running full correct color on this same
 * physical unit, and (2) after fixing the RGB panel timing + using the
 * board-specific init sequence + real per-transaction CS toggling, a solid
 * WHITE fill renders as true white — meaning all three channels are fully
 * healthy. What remains is a clean 3-way channel rotation confirmed via the
 * RGB lane test: transmitted R drives the physical blue subpixel,
 * transmitted G drives physical red, transmitted B drives physical green.
 * remap_pixel() below corrects this rotation in software.
 */

// WiFi credentials are handled entirely by WiFiManager (captive portal on
// first boot / whenever the stored network can't be reached) — no longer
// hardcoded here. NAS host/port/user/password live in NVS (see
// loadSiteConfig()/saveSiteConfig()) and are set via that same portal, or
// anytime afterward via the web UI's "Settings" page. The values below
// are only placeholders used the very first time a device with blank NVS
// boots this firmware — real credentials for any given site should never
// be written here; set them through the portal/Settings page instead.
#define DEFAULT_NAS_HOST     "192.168.1.1"
#define DEFAULT_DSM_PORT     "5000"
#define DEFAULT_DSM_USER     ""
#define DEFAULT_DSM_PASSWORD ""
#define SCREENSAVER_TIMEOUT_MS  30000UL
#define DEBOUNCE_MS             20000UL
#define MAX_JPEG_BYTES (1536 * 1024)  // was 512KB — too small for one high-res camera at
    // 2880x1620, which sends ~956KB JPEGs. The old cap silently truncated the read
    // (readBytes stops exactly at the cap with no error), producing a valid-looking partial
    // JPEG that decoded fine for its first rows then went to garbage once the entropy-coded
    // data ran out — confirmed via a missing EOI marker in the [SS] diagnostic log. Was bumped
    // to 2MB after that fix; trimmed back to 1.5MB (still ~600KB/64% of margin above the
    // observed 956KB worst case) to reclaim PSRAM for a deeper history ring (HISTORY_SLOTS).
    // snapBuf shares this cap (it only ever holds a copy of what jpegBuf received), so this
    // trim reclaims from both buffers at once.

// Set to 1 to boot straight into a solid-fill RGB lane test instead of
// normal operation. Confirms the pin-order fix below before trusting real
// JPEG color decode. Flip back to 0 once R/G/B/W/black all render correctly.
#define COLOR_LANE_TEST 0

// Set to 1 to boot straight into a synthetic high-detail test pattern
// (fine checkerboard + gradient), bypassing JPEG/camera entirely, to test
// whether the display hardware itself corrupts fine-detail content even
// with zero involvement from the JPEG decode chain.
#define GRADIENT_TEST 0

#define SCREEN_W     480
#define SCREEN_H     480
#define LCD_DE       18
#define LCD_VS       17
#define LCD_HS       16
#define LCD_PCLK     21
#define LCD_SPI_SCK  41
#define LCD_SPI_MOSI 48
#define LCD_BL       45
#define I2C_SDA      39
#define I2C_SCL      40
#define TOUCH_I2C_ADDR 0x48  // FT6336U capacitive touch controller, same I2C bus as the PCA9535

// Single green channel only: bits[10:8] → G sub-pixel. All colours are shades of green.
// bigEndian=true + old pins + MADCTL=0x08 confirmed: 0x0700→GREEN, 0x0007→BLACK.
#define COL_BLACK   0x0000
#define COL_WHITE   0x0700  // max green (3-bit lum=7)
#define COL_GREY    0x0400  // mid green  (lum=4)
#define COL_RED     0x0700  // alert = bright green (no red available)
#define COL_GREEN   0x0700
#define COL_ORANGE  0x0500
#define COL_DIMBAR  0x0100

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Arduino_GFX_Library.h>
#include <PCA95x5.h>
#include <JPEGDEC.h>
#include <TJpg_Decoder.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <time.h>
#include "font/glcdfont.h"  // GFX library's built-in 5x7 font; see drawTextRaw()

// Site-specific config (NAS host/port/user/password), loaded from NVS at
// boot via loadSiteConfig() and edited through the WiFiManager portal /
// web UI — see loadSiteConfig()/saveSiteConfig() near setup().
Preferences g_configPrefs;
String g_nasHost, g_dsmUser, g_dsmPassword;
int    g_dsmPort;

// Optional second NAS ("site 2") for a Surveillance Station instance at a
// different location (e.g. reached over a Tailscale link), so cameras from
// both sites can be resolved and fetched from a single device. Disabled
// whenever g_nasHost2 is empty — every site-2 code path below is gated on
// site2Configured(). A webhook's camera name is looked up across both
// sites' camera lists (see cameraIdForName()) — relies on camera names
// being unique across the two NASes, since nothing else disambiguates them.
String g_nasHost2, g_dsmUser2, g_dsmPassword2;
int    g_dsmPort2;

bool site2Configured() { return g_nasHost2.length() > 0; }

void loadSiteConfig() {
    g_configPrefs.begin("sitecfg", true);
    g_nasHost     = g_configPrefs.getString("nasHost", DEFAULT_NAS_HOST);
    g_dsmPort     = g_configPrefs.getInt("dsmPort", atoi(DEFAULT_DSM_PORT));
    g_dsmUser     = g_configPrefs.getString("dsmUser", DEFAULT_DSM_USER);
    g_dsmPassword = g_configPrefs.getString("dsmPass", DEFAULT_DSM_PASSWORD);
    g_nasHost2     = g_configPrefs.getString("nasHost2", "");
    g_dsmPort2     = g_configPrefs.getInt("dsmPort2", atoi(DEFAULT_DSM_PORT));
    g_dsmUser2     = g_configPrefs.getString("dsmUser2", "");
    g_dsmPassword2 = g_configPrefs.getString("dsmPass2", "");
    g_configPrefs.end();
}

void saveSiteConfig() {
    g_configPrefs.begin("sitecfg", false);
    g_configPrefs.putString("nasHost", g_nasHost);
    g_configPrefs.putInt("dsmPort", g_dsmPort);
    g_configPrefs.putString("dsmUser", g_dsmUser);
    g_configPrefs.putString("dsmPass", g_dsmPassword);
    g_configPrefs.putString("nasHost2", g_nasHost2);
    g_configPrefs.putInt("dsmPort2", g_dsmPort2);
    g_configPrefs.putString("dsmUser2", g_dsmUser2);
    g_configPrefs.putString("dsmPass2", g_dsmPassword2);
    g_configPrefs.end();
}

// On-screen overlay preferences (camera name / date-time visibility, clock
// format, date field order), loaded from NVS at boot and edited via the
// web UI's "Settings" page alongside the NAS connection fields.
bool   g_showCamName  = true;
bool   g_showDateTime = true;
bool   g_use24Hour    = true;
String g_dateFormat   = "ymd";  // one of "ymd", "mdy", "dmy"

void loadDisplayConfig() {
    g_configPrefs.begin("dispcfg", true);
    g_showCamName  = g_configPrefs.getBool("showCam", true);
    g_showDateTime = g_configPrefs.getBool("showTime", true);
    g_use24Hour    = g_configPrefs.getBool("use24h", true);
    g_dateFormat   = g_configPrefs.getString("dateFmt", "ymd");
    g_configPrefs.end();
}

void saveDisplayConfig() {
    g_configPrefs.begin("dispcfg", false);
    g_configPrefs.putBool("showCam", g_showCamName);
    g_configPrefs.putBool("showTime", g_showDateTime);
    g_configPrefs.putBool("use24h", g_use24Hour);
    g_configPrefs.putString("dateFmt", g_dateFormat);
    g_configPrefs.end();
}

// Set to 1 to decode with TJpg_Decoder (a completely independent JPEG
// library) instead of JPEGDEC, keeping everything downstream (remap_pixel,
// blitWindow, double buffering) identical — isolates whether JPEGDEC
// itself is the source of the persistent detail-correlated corruption.
#define USE_TJPG_DECODER 1

// The old gfx->flush() (removed when switching to presentBuffer() for
// double buffering) called this to force cached CPU writes to actually
// commit to PSRAM before the display hardware reads it. presentBuffer()
// doesn't do this itself — missing it would mean the panel could read
// stale/garbage data mid-write, which matches the corruption pattern
// exactly (confirmed the raw JPEG file itself is clean, so this is the
// next real candidate).
extern "C" int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

PCA9535 ioex;

// Exact copy of st7701_sensecap_indicator_init_operations from openHASP's
// own patched GFX library fork (lib/Arduino_RPi_DPI_RGBPanel_mod/
// Arduino_RGB_Display_mod.h), used specifically for `defined(SENSECAP_INDICATOR_D1)`
// in their tft_driver_arduinogfx.cpp — a completely separate, dedicated
// board branch from the generic ST7701_DRIVER/type1 one we'd been comparing
// against. Key difference: MADCTL (0x36) = 0x10 here, not 0x08 — that's the
// real, board-specific value, not a generic ST7701 default.
static const uint8_t sensecap_init[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x10,
    WRITE_C8_D16, 0xC0, 0x3B, 0x00,
    WRITE_C8_D16, 0xC1, 0x0D, 0x02,
    WRITE_C8_D16, 0xC2, 0x31, 0x05,
    WRITE_C8_D8, 0xC7, 0x04,
    WRITE_C8_D8, 0xCD, 0x08,
    WRITE_COMMAND_8, 0xB0,
    WRITE_BYTES, 16, 0x00, 0x11, 0x18, 0x0E, 0x11, 0x06, 0x07, 0x08,
                     0x07, 0x22, 0x04, 0x12, 0x0F, 0xAA, 0x31, 0x18,
    END_WRITE,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xB1,
    WRITE_BYTES, 16, 0x00, 0x11, 0x19, 0x0E, 0x12, 0x07, 0x08, 0x08,
                     0x08, 0x22, 0x04, 0x11, 0x11, 0xA9, 0x32, 0x18,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x11,
    WRITE_C8_D8, 0xB0, 0x60,
    WRITE_C8_D8, 0xB1, 0x32,
    WRITE_C8_D8, 0xB2, 0x07,
    WRITE_C8_D8, 0xB3, 0x80,
    WRITE_C8_D8, 0xB5, 0x49,
    WRITE_C8_D8, 0xB7, 0x85,
    WRITE_C8_D8, 0xB8, 0x21,
    WRITE_C8_D8, 0xC1, 0x78,
    WRITE_C8_D8, 0xC2, 0x78,
    END_WRITE,
    DELAY, 20,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xE0,
    WRITE_BYTES, 3, 0x00, 0x1B, 0x02,
    WRITE_COMMAND_8, 0xE1,
    WRITE_BYTES, 11, 0x08, 0xA0, 0x00, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x44, 0x44,
    WRITE_COMMAND_8, 0xE2,
    WRITE_BYTES, 12, 0x11, 0x11, 0x44, 0x44, 0xED, 0xA0, 0x00, 0x00, 0xEC, 0xA0, 0x00, 0x00,
    END_WRITE,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xE3,
    WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,
    WRITE_C8_D16, 0xE4, 0x44, 0x44,
    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 16, 0x0A, 0xE9, 0xD8, 0xA0, 0x0C, 0xEB, 0xD8, 0xA0,
                     0x0E, 0xED, 0xD8, 0xA0, 0x10, 0xEF, 0xD8, 0xA0,
    WRITE_COMMAND_8, 0xE6,
    WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,
    WRITE_C8_D16, 0xE7, 0x44, 0x44,
    END_WRITE,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xE8,
    WRITE_BYTES, 16, 0x09, 0xE8, 0xD8, 0xA0, 0x0B, 0xEA, 0xD8, 0xA0,
                     0x0D, 0xEC, 0xD8, 0xA0, 0x0F, 0xEE, 0xD8, 0xA0,
    WRITE_COMMAND_8, 0xEB,
    WRITE_BYTES, 7, 0x02, 0x00, 0xE4, 0xE4, 0x88, 0x00, 0x40,
    WRITE_C8_D16, 0xEC, 0x3C, 0x00,
    WRITE_COMMAND_8, 0xED,
    WRITE_BYTES, 16, 0xAB, 0x89, 0x76, 0x54, 0x02, 0xFF, 0xFF, 0xFF,
                     0xFF, 0xFF, 0xFF, 0x20, 0x45, 0x67, 0x98, 0xBA,
    WRITE_C8_D8, 0x36, 0x10,
    END_WRITE,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x13,
    WRITE_C8_D8, 0xE5, 0xE4,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x00,
    WRITE_COMMAND_8, 0x21,
    WRITE_C8_D8, 0x3A, 0x60,
    WRITE_COMMAND_8, 0x11,
    END_WRITE,
    DELAY, 120,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x29,
    END_WRITE,
    DELAY, 120,
};

// openHASP's own SenseCAP bus class (Arduino_PCA9535SWSPI) toggles the real
// IO-expander CS pin on every beginWrite()/endWrite() — i.e. around each
// BEGIN_WRITE/END_WRITE block in the init sequence, not just once for the
// whole thing. Our plain Arduino_SWSPI (CS=GFX_NOT_DEFINED) never touches
// CS at all; we were holding it low via the IO expander for the entire init
// burst instead of per-command, which likely desyncs this bit-banged,
// DC-less protocol's command framing — a strong candidate for the
// persistent channel-rotated colors even with byte-for-byte correct
// register values. This subclass adds real per-transaction CS toggling on
// top of Arduino_SWSPI's already-correct bit-bang implementation.
class Arduino_SWSPI_PCA9535CS : public Arduino_SWSPI {
public:
    Arduino_SWSPI_PCA9535CS(int8_t sck, int8_t mosi)
        : Arduino_SWSPI(GFX_NOT_DEFINED, GFX_NOT_DEFINED, sck, mosi, GFX_NOT_DEFINED) {}

    void beginWrite() override {
        ioex.write(PCA95x5::Port::P04, PCA95x5::Level::L);
        Arduino_SWSPI::beginWrite();
    }
    void endWrite() override {
        Arduino_SWSPI::endWrite();
        ioex.write(PCA95x5::Port::P04, PCA95x5::Level::H);
    }
};

Arduino_DataBus *bus = new Arduino_SWSPI_PCA9535CS(LCD_SPI_SCK, LCD_SPI_MOSI);

// Reverted to useBigEndian=true / pclk_active_neg=1 / prefer_speed=18000000
// — the useBigEndian=false + bit-scramble-correct combination both passed
// every synthetic color test but produced chaotic rainbow noise on real,
// rapidly-varying JPEG content (twice, with two different remap_pixel
// approaches), while this exact configuration is the only one so far that
// renders real photos cleanly (structure intact, no corruption) — pointing
// at a timing/signal-integrity issue under real content rather than a
// channel-mapping logic bug. Color balance still needs a remap_pixel fix
// (see below) but corruption-free structure comes first.
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    LCD_DE, LCD_VS, LCD_HS, LCD_PCLK,
    4, 3, 2, 1, 0,        // r0..r4
    10, 9, 8, 7, 6, 5,    // g0..g5
    15, 14, 13, 12, 11,   // b0..b4
    1, 10, 8, 50, 1, 10, 8, 20,
    0, 12000000, false, 0, 0, 0);  // TEST: exactly matches openHASP's actual SENSECAP_INDICATOR_D1
    // call (tft_driver_arduinogfx.cpp) — pclk_active_neg=0 (was 1), useBigEndian=false (was true),
    // prefer_speed=12MHz matching the octal-PSRAM default (was 6/3MHz), no bounce buffer (was
    // 480*60). This exact combination was never tested together before.

// rotation=2 was tried to fix upside-down boot/status text (Arduino_GFX's
// own fillScreen/print path, separate from our direct-pixel blitWindow/
// blitFit) but had no visible effect — reverted to 0. presentG_fb() now
// does a manual 180° flip of fbBuf[0] instead, since that's the one thing
// actually proven to affect what's on screen.
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    SCREEN_W, SCREEN_H, rgbpanel, 0, true,
    bus, GFX_NOT_DEFINED,
    sensecap_init, sizeof(sensecap_init));

// Simple channel-rotation correction, confirmed via the color-lane test
// (fillScreen path): with useBigEndian=true, sending pure R lights physical
// blue, pure G lights physical red, pure B lights physical green. This
// corrects that — real photos render with correct structure and only a
// blue/cyan color-balance skew. A desaturation post-process was tried on
// top of this and produced the worst corruption seen all session, despite
// being simple bounded arithmetic that shouldn't be able to cause it —
// reverted. Leaving color balance as-is rather than risk further attempts.
// TEST: openHASP's actual constructor call for SENSECAP_INDICATOR_D1 (in
// tft_driver_arduinogfx.cpp) never passes pclk_active_neg/prefer_speed/
// useBigEndian — they silently default to 0/GFX_NOT_DEFINED(->12MHz for
// octal PSRAM)/false. We had been running pclk_active_neg=1, useBigEndian=
// true, plus a bounce buffer — a combination never verified against this
// exact reference baseline. Matching it means standard RGB565 needs no
// channel-rotation correction at all, so this is now the identity function.
static inline uint16_t remap_pixel(uint16_t p) {
    return p;
}

// Bumped from 2MB to fit half-scale decode (1440x810x2 = ~2.33MB) instead
// of falling back to quarter-scale — testing whether the corruption we've
// been chasing is specific to JPEGDEC's quarter-scale IDCT path.
// Was 3MB — real decode targets for this project's (all 16:9) cameras top
// out around 720x405 (~570KB) after the dynamic downscale-to-fit-screen-
// width logic below runs, so 3MB left ~2.4MB of PSRAM sitting idle. Trimmed
// to 1.5MB, then to 1MB (still ~1.8x the observed ~570KB worst case) to
// reclaim more room when HISTORY_SLOTS went from 3 to 5 — the existing
// downscale-until-it-fits loop already handles gracefully falling back
// further if some future camera needs more than this cap allows.
#define MAX_DECODE_BYTES (1024 * 1024)

JPEGDEC   jpeg;
uint8_t  *jpegBuf    = nullptr;
uint8_t  *snapBuf    = nullptr;
size_t    snapLen    = 0;
int       snapCam    = 0;
time_t    snapTime   = 0;  // wall-clock time the buffered snapshot was fetched,
                           // not whenever it happens to be redrawn (e.g. tap-to-wake)
uint16_t *g_fb       = nullptr;   // == fbBuf[0], kept for the existing null-check
uint16_t *decodeBuf  = nullptr;
int       decodedW   = 0;
int       decodedH   = 0;

// On-device photo history: a small ring of fully-composited frames (image +
// letterbox bars + camera name/date-time overlay, exactly as shown on
// screen), so browsing backward needs no network fetch or JPEG re-decode —
// just a memcpy + presentBuffer(). Sized at 6 slots (see MAX_JPEG_BYTES /
// MAX_DECODE_BYTES comments above for where the PSRAM to fit this came
// from) — snapBuf above stays sized off MAX_JPEG_BYTES regardless, since it
// independently backs the /snapshot HTTP endpoint. Bumped from 5 to 6:
// at 7,451,364 bytes of free PSRAM measured just before these allocations
// (see [BOOT] spiram_cap log line in setup()), 6 slots leaves ~490KB of
// PSRAM headroom vs. ~30KB at the hard ceiling of 7 slots — kept back from
// the ceiling on purpose, same margin-over-exact-fit practice as the
// MAX_JPEG_BYTES/MAX_DECODE_BYTES trims above.
#define HISTORY_SLOTS 6
uint16_t *g_historyBuf[HISTORY_SLOTS]        = {};  // zero/null-initializes every element
int       g_historyBarCenterY[HISTORY_SLOTS] = {};  // that slot's arrow/date-time row, stored
                                                     // per-slot in case a future camera's aspect ratio differs
int       g_historyCount  = 0;   // slots populated so far, caps at HISTORY_SLOTS
int       g_historyHead   = -1;  // ring index of the most recently pushed frame
int       g_historyOffset = 0;   // 0 = live/most recent; larger = further back
int       g_barCenterY    = SCREEN_H - 40;  // vertical center of the arrow/date-time
                                             // row, updated each render; used for touch hit-testing

// Double buffering: patched Arduino_ESP32RGBPanel to expose both driver
// buffers + a vsync-synced present() (esp_lcd_panel_draw_bitmap), instead
// of memcpy-ing directly into the single live buffer the DMA continuously
// scans out. That race (invisible on solid colors, catastrophic on real
// varying photo content) is what all the "rainbow chaos" this session was.
uint16_t *fbBuf[2]  = { nullptr, nullptr };
int       activeBuf = 0;  // index currently being displayed
int       lastCamId = 0;  // camera id of the frame currently on screen

// id->name lookup populated by ssListCameras() (see below), used by blitFit()
// to label the on-screen image. Declared this early (rather than next to
// ssListCameras() itself) so blitFit() — which is defined before the
// networking code — can reference it.
// Bumped from 16 to 32 to hold both sites' cameras combined (see
// site2Configured() above) — 16 per site.
#define MAX_CAMERAS 32
// Each id here is a *combined* id: site (0 or 1) packed into bit 16, the
// Synology-native camera id in the low 16 bits — see ssListCameras() and
// ssGetSnapshot(). Synology ids are small per-NAS integers, so two sites'
// ids can otherwise collide; packing the site in keeps every id in this
// table globally unique and lets ssGetSnapshot() route to the right NAS
// without a separate lookup table.
int    g_camIds[MAX_CAMERAS];
String g_camNames[MAX_CAMERAS];
int    g_camCount = 0;

String cameraName(int id) {
    for (int i = 0; i < g_camCount; i++) if (g_camIds[i] == id) return g_camNames[i];
    return "Cam " + String(id & 0xFFFF);
}

// Reverse of cameraName() — case-insensitive match against the cached
// camera list, so Action Rule webhooks can use Synology's "device name"
// ingredient (there's no "camera ID" ingredient) instead of a hardcoded id.
//
// Names with spaces have been unreliable in practice (single-word names
// work every time) — suspected cause is Synology's ingredient
// substitution not URL-encoding the space, and a raw space being structurally
// significant in an HTTP request line (it can truncate/mangle the request
// before it reaches us). Try a '+'-as-space fallback (in case it came
// through form-encoded instead of percent-encoded) before giving up.
int cameraIdForName(const String &name) {
    // Searches both sites' cameras combined — relies on camera names being
    // unique across the two NASes (see g_nasHost2 comment above).
    for (int i = 0; i < g_camCount; i++) {
        if (g_camNames[i].equalsIgnoreCase(name)) return g_camIds[i];
    }
    String withSpaces = name;
    withSpaces.replace('+', ' ');
    if (withSpaces != name) {
        for (int i = 0; i < g_camCount; i++) {
            if (g_camNames[i].equalsIgnoreCase(withSpaces)) return g_camIds[i];
        }
    }
    return -1;
}

// Accepts either a plain numeric camera id (old-style rules, e.g. "cam=1")
// or a camera name (from Synology's "device name" ingredient, e.g.
// "cam=Living Room") and resolves either to a numeric id. Returns -1 if
// the value is empty or names a camera we don't recognize.
int parseCamParam(const String &raw) {
    String v = raw;
    v.trim();
    if (v.isEmpty()) return -1;
    bool numeric = true;
    for (size_t i = 0; i < v.length(); i++) {
        if (!isDigit((unsigned char)v[i])) { numeric = false; break; }
    }
    if (numeric) return v.toInt();
    return cameraIdForName(v);
}

// gfx->fillScreen()/print() (boot messages, errors, screensaver) always
// write to fbBuf[0] internally (Arduino_RGB_Display caches one fixed
// pointer) — call this right after any such draw so it actually becomes
// visible and our own tracking stays in sync, regardless of which buffer
// blitWindow() last swapped to.
void presentG_fb() {
    // TEST: manual flip removed. A leftover gfx->setRotation(2) call (just
    // removed) was silently forcing rotation=2 on every prior test in this
    // area, including when the constructor's own rotation arg was believed
    // to be 0 — so "no manual transform at all" has never actually been
    // tested until now. Re-add a flip here only if this comes out upside
    // down, informed by what this test actually shows.
    Cache_WriteBack_Addr((uint32_t)fbBuf[0], SCREEN_W * SCREEN_H * 2);
    rgbpanel->presentBuffer(fbBuf[0], SCREEN_W, SCREEN_H);
    activeBuf = 0;
}

// Pan state
int           panYOff   = 0;
bool          panLeft   = true;
unsigned long lastPanMs = 0;

// Debug instrumentation: track what JPEGDEC actually reports per callback
// so we can verify it matches our decodedW/decodedH assumptions.
int dbgCbCount = 0, dbgMaxX = 0, dbgMaxY = 0, dbgMaxIW = 0, dbgMaxIH = 0;

// Decode into decodeBuf (no flip — blitWindow handles that)
int jpegDrawCB(JPEGDRAW *d) {
    if (!decodeBuf) return 0;
    dbgCbCount++;
    if (d->x > dbgMaxX) dbgMaxX = d->x;
    if (d->y > dbgMaxY) dbgMaxY = d->y;
    if (d->iWidth  > dbgMaxIW) dbgMaxIW  = d->iWidth;
    if (d->iHeight > dbgMaxIH) dbgMaxIH  = d->iHeight;
    uint16_t *src = d->pPixels;
    // Diagnostic: dump raw JPEGDEC output (pre-remap) for a handful of
    // sample points so we can inspect what byte/bit convention
    // RGB565_BIG_ENDIAN actually produces, vs. the native hex constants
    // the color-lane test (fillScreen path) used to derive remap_pixel.
    static const int sampleX[] = { 100, 360, 600, 360, 360 };
    static const int sampleY[] = { 200, 50,  200, 200, 350 };
    for (int row = 0; row < d->iHeight; row++) {
        int y = d->y + row;
        if (y < decodedH) {
            uint16_t *dst = decodeBuf + y * decodedW + d->x;
            int w = d->iWidth;
            if (d->x + w > decodedW) w = decodedW - d->x;
            for (int i = 0; i < w; i++) {
                // Byte-swap REMOVED: it was based on assuming a few sample
                // pixels must be neutral gray without ever verifying that
                // against the true source photo. Every corrupted/chaotic
                // real-photo result this session came AFTER this swap was
                // added; the double-buffering fix (mechanically confirmed
                // working) made zero difference, which points back here —
                // JPEGDEC's RGB565_BIG_ENDIAN output was likely already
                // correctly ordered, and this extra swap was scrambling it.
                uint16_t rawVal = src[i];
                dst[i] = remap_pixel(rawVal);
                int px = d->x + i;
                for (int s = 0; s < 5; s++) {
                    if (px == sampleX[s] && y == sampleY[s]) {
                        uint16_t rr = (rawVal >> 11) & 0x1F;
                        uint16_t gg = (rawVal >> 5)  & 0x3F;
                        uint16_t bb = rawVal & 0x1F;
                        Serial.printf("[PXDBG] (%d,%d) swapped=0x%04X r=%d g=%d b=%d remapped=0x%04X\n",
                            px, y, rawVal, rr, gg, bb, dst[i]);
                    }
                }
            }
        }
        src += d->iWidth;
    }
    return 1;
}

#if USE_TJPG_DECODER
// TJpg_Decoder's output callback — independent decoder, same downstream
// pipeline (remap_pixel + decodeBuf) as jpegDrawCB above.
// Ground-truth comparison points, sampled from the same snapshot on a PC:
// (50,150)=0x52AA (200,300)=0x9D14 (400,200)=0xD73E (100,550)=0x6B4C
// (300,400)=0x3986 (150,250)=0x7BCE (350,350)=0x3A06 -- all RGB565, pre-remap.
static const int gtX[] = {50,200,400,100,300,150,350};
static const int gtY[] = {150,300,200,550,400,250,350};
bool tjpgOutputCB(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
    if (!decodeBuf) return false;
    for (int row = 0; row < h; row++) {
        int py = y + row;
        if (py < 0 || py >= decodedH) continue;
        uint16_t *dst = decodeBuf + py * decodedW;
        for (int col = 0; col < w; col++) {
            int px = x + col;
            if (px < 0 || px >= decodedW) continue;
            uint16_t raw = bitmap[row * w + col];
            dst[px] = remap_pixel(raw);
            for (int s = 0; s < 7; s++) {
                if (px == gtX[s] && py == gtY[s]) {
                    Serial.printf("[GT] (%d,%d) decoded_raw=0x%04X\n", px, py, raw);
                }
            }
        }
    }
    return true;
}
#endif

// Blit a SCREEN_W × SCREEN_H window from decodeBuf to the display.
// Writes into whichever buffer isn't currently on-screen, then presents it
// via presentBuffer() (vsync-synced swap) instead of memcpy-ing into the
// live, actively-scanned buffer — eliminates the tearing race that showed
// up as chaotic corruption on real photo content all session.
#define BLIT_FLIP_180 0
void blitWindow(int xOff, int yOff) {
    int target = 1 - activeBuf;
    uint16_t *destFb = fbBuf[target];
    uint16_t rowbuf[SCREEN_W];
    int srcH = min(SCREEN_H, decodedH - yOff);
    int srcW = min(SCREEN_W, decodedW - xOff);
    memset(destFb, 0, SCREEN_W * SCREEN_H * 2);
    for (int y = 0; y < srcH; y++) {
        uint16_t *src_row = decodeBuf + (yOff + y) * decodedW + xOff;
#if BLIT_FLIP_180
        int dst_y = SCREEN_H - 1 - y;  // Y-flip
        for (int x = 0; x < srcW; x++)
            rowbuf[SCREEN_W - 1 - x] = src_row[x];  // X-flip into local buf
#else
        int dst_y = y;
        for (int x = 0; x < srcW; x++)
            rowbuf[x] = src_row[x];
#endif
        memcpy(destFb + dst_y * SCREEN_W, rowbuf, SCREEN_W * 2);
    }
    // Same ground-truth sample points as tjpgOutputCB's [GT] log, but read
    // back from destFb (post-blit, pre-present) to isolate whether blitWindow's
    // copy corrupts data that decodeBuf already had correct.
    for (int s = 0; s < 7; s++) {
        int sx = gtX[s] - xOff, sy = gtY[s] - yOff;
        if (sx >= 0 && sx < SCREEN_W && sy >= 0 && sy < srcH) {
            Serial.printf("[GT-FB] (%d,%d) fbBuf=0x%04X\n", gtX[s], gtY[s], destFb[sy * SCREEN_W + sx]);
        }
    }
    Cache_WriteBack_Addr((uint32_t)destFb, SCREEN_W * SCREEN_H * 2);
    bool ok = rgbpanel->presentBuffer(destFb, SCREEN_W, SCREEN_H);
    Serial.printf("[BLIT] wrote fbBuf[%d]=%p, presentBuffer=%s\n", target, destFb, ok ? "OK" : "FAIL");
    activeBuf = target;
}

// Draws text straight into an arbitrary destination framebuffer using the
// GFX library's built-in 5x7 font, bypassing Arduino_GFX's own text renderer
// (gfx->print()) — which, per the note above presentG_fb(), always targets
// the fixed fbBuf[0] regardless of which buffer is about to be presented.
// Drawing here instead guarantees the overlay lands on the same buffer as
// the image frame it's labeling.
void drawTextRaw(uint16_t *fb, int x, int y, const String &text, uint16_t color, int scale) {
    int cx = x;
    for (size_t n = 0; n < text.length(); n++) {
        uint8_t c = (uint8_t)text[n];
        for (int col = 0; col < 5; col++) {
            uint8_t line = pgm_read_byte(&font[c * 5 + col]);
            for (int row = 0; row < 8; row++, line >>= 1) {
                if (!(line & 1)) continue;
                for (int sy = 0; sy < scale; sy++) {
                    int py = y + row * scale + sy;
                    if (py < 0 || py >= SCREEN_H) continue;
                    for (int sx = 0; sx < scale; sx++) {
                        int px = cx + col * scale + sx;
                        if (px < 0 || px >= SCREEN_W) continue;
                        fb[py * SCREEN_W + px] = color;
                    }
                }
            }
        }
        cx += 6 * scale;  // 5px glyph + 1px gap, scaled
    }
}

int textWidthRaw(const String &text, int scale) {
    return (int)text.length() * 6 * scale;
}

// Copies a freshly-composited frame (image + camera name/date-time overlay,
// but NOT the history arrows — those are drawn fresh on every display, see
// drawHistoryArrows()) into the history ring, becoming the new "live"
// position. Only called for genuinely new captures (from blitFit(), once
// per real fetch) — never for history-browsing redraws, so old frames never
// get pushed back in as "new".
void pushHistory(uint16_t *frame, int barCenterY) {
    g_historyHead = (g_historyHead + 1) % HISTORY_SLOTS;
    memcpy(g_historyBuf[g_historyHead], frame, SCREEN_W * SCREEN_H * 2);
    g_historyBarCenterY[g_historyHead] = barCenterY;
    if (g_historyCount < HISTORY_SLOTS) g_historyCount++;
    g_historyOffset = 0;
}

int historyRingIndex(int offset) {
    return (g_historyHead - offset + HISTORY_SLOTS) % HISTORY_SLOTS;
}

// Draws only the arrow(s) that are actually valid to tap from the given
// history offset — hidden entirely (not just inert) at either end so
// there's no way to page past the oldest frame or past "live". Called both
// right after a fresh capture (blitFit()) and on every history redraw
// (showHistoryOffset()), so the arrows always reflect where you currently
// are, not whatever was true when that particular frame was captured.
void drawHistoryArrows(uint16_t *fb, int barCenterY, int offset) {
    const int arrowScale = 3;
    int arrowY = barCenterY - (8 * arrowScale) / 2;
    bool canGoOlder = offset < g_historyCount - 1;
    bool canGoNewer = offset > 0;
    if (canGoOlder) drawTextRaw(fb, 4, arrowY, "<", COL_WHITE, arrowScale);
    if (canGoNewer) drawTextRaw(fb, SCREEN_W - 4 - textWidthRaw(">", arrowScale), arrowY, ">", COL_WHITE, arrowScale);
}

// Redisplays an already-composited frame straight from the history ring —
// no decode, no re-render, just a copy into the inactive framebuffer and a
// present, so paging through history is effectively instant.
void showHistoryOffset(int offset) {
    if (g_historyCount == 0) return;
    if (offset < 0) offset = 0;
    if (offset > g_historyCount - 1) offset = g_historyCount - 1;
    g_historyOffset = offset;
    int idx = historyRingIndex(offset);
    int target = 1 - activeBuf;
    memcpy(fbBuf[target], g_historyBuf[idx], SCREEN_W * SCREEN_H * 2);
    g_barCenterY = g_historyBarCenterY[idx];
    drawHistoryArrows(fbBuf[target], g_barCenterY, offset);
    Cache_WriteBack_Addr((uint32_t)fbBuf[target], SCREEN_W * SCREEN_H * 2);
    rgbpanel->presentBuffer(fbBuf[target], SCREEN_W, SCREEN_H);
    activeBuf = target;
}

// Whole-frame letterboxed fit — no cropping or left/right panning. Scales
// decodeBuf down (nearest-neighbor) to fill either the full width or full
// height of the screen, whichever the source aspect ratio constrains, and
// centers it with black bars on the other axis.
void blitFit() {
    int target = 1 - activeBuf;
    uint16_t *destFb = fbBuf[target];
    memset(destFb, 0, SCREEN_W * SCREEN_H * 2);

    int fitW, fitH;
    if ((long)decodedW * SCREEN_H >= (long)decodedH * SCREEN_W) {
        fitW = SCREEN_W;
        fitH = (int)((long)decodedH * SCREEN_W / decodedW);
    } else {
        fitH = SCREEN_H;
        fitW = (int)((long)decodedW * SCREEN_H / decodedH);
    }
    int xOff = (SCREEN_W - fitW) / 2;
    int yOff = (SCREEN_H - fitH) / 2;

    uint16_t rowbuf[SCREEN_W];
    for (int dy = 0; dy < fitH; dy++) {
        int srcY = (int)((long)dy * decodedH / fitH);
        uint16_t *srcRow = decodeBuf + srcY * decodedW;
        for (int dx = 0; dx < fitW; dx++) {
            int srcX = (int)((long)dx * decodedW / fitW);
            rowbuf[dx] = srcRow[srcX];
        }
        memcpy(destFb + (yOff + dy) * SCREEN_W + xOff, rowbuf, fitW * 2);
    }

    // Camera name centered in the top letterbox bar, date/time centered in
    // the bottom one — each independently toggleable, per g_showCamName /
    // g_showDateTime (see loadDisplayConfig()). Falls back to overlaying on
    // the image itself (rather than skipping the label) on a near-square
    // source with little/no bar.
    const int scale = 2;
    const int textH = 8 * scale;

    if (g_showCamName) {
        String camName = cameraName(lastCamId);
        int camX = (SCREEN_W - textWidthRaw(camName, scale)) / 2;
        int camY = (yOff - textH) / 2;
        if (camY < 2) camY = 2;
        drawTextRaw(destFb, camX, camY, camName, COL_WHITE, scale);
    }

    // Computed unconditionally (not just under g_showDateTime) since the
    // history arrow buttons share this same row regardless of whether the
    // date/time label itself is shown.
    int bottomBarH = SCREEN_H - yOff - fitH;
    int barCenterY = yOff + fitH + bottomBarH / 2;

    if (g_showDateTime) {
        String dateFmt = g_dateFormat == "mdy" ? "%m-%d-%Y"
                        : g_dateFormat == "dmy" ? "%d-%m-%Y"
                        : "%Y-%m-%d";
        String timeFmt = g_use24Hour ? "%H:%M:%S" : "%I:%M:%S %p";
        String fmt = dateFmt + " " + timeFmt;
        struct tm timeinfo;
        localtime_r(&snapTime, &timeinfo);
        char timeBuf[32];
        strftime(timeBuf, sizeof(timeBuf), fmt.c_str(), &timeinfo);
        String timeStr(timeBuf);
        int timeX = (SCREEN_W - textWidthRaw(timeStr, scale)) / 2;
        int timeY = yOff + fitH + (bottomBarH - textH) / 2;
        if (timeY > SCREEN_H - textH - 2) timeY = SCREEN_H - textH - 2;
        drawTextRaw(destFb, timeX, timeY, timeStr, COL_WHITE, scale);
    }

    // Push to history BEFORE drawing arrows, so the archived copy holds
    // just the image + labels — arrows get redrawn fresh every time any
    // frame (live or historical) is displayed, since whether each arrow
    // should even be visible depends on the current browsing position, not
    // on whatever was true at capture time.
    g_barCenterY = barCenterY;
    pushHistory(destFb, barCenterY);

    // History-browsing arrows, on the same row as the date/time bar,
    // pinned to the screen edges — independent of g_showDateTime since
    // they're navigation controls, not the label. Hit-testing against
    // these lives in loop() via g_barCenterY. A fresh capture always lands
    // at offset 0 (live), so the "newer" arrow is correctly absent here.
    drawHistoryArrows(destFb, barCenterY, g_historyOffset);

    Cache_WriteBack_Addr((uint32_t)destFb, SCREEN_W * SCREEN_H * 2);
    bool ok = rgbpanel->presentBuffer(destFb, SCREEN_W, SCREEN_H);
    Serial.printf("[BLIT-FIT] wrote fbBuf[%d]=%p, %dx%d @ (%d,%d), presentBuffer=%s\n",
        target, destFb, fitW, fitH, xOff, yOff, ok ? "OK" : "FAIL");
    activeBuf = target;
}

String g_sid = "";   // site 0 (primary/home) session id
String g_sid2 = "";  // site 1 (site2Configured()) session id
String g_camerasJson = "";  // cached JSON array of {id,name} from the last successful List call, both sites combined

String nasApiUrl(int site) {
    if (site == 1) return "http://" + g_nasHost2 + ":" + String(g_dsmPort2) + "/webapi/";
    return "http://" + g_nasHost + ":" + String(g_dsmPort) + "/webapi/";
}

bool ssLogin(int site) {
    String &sid  = (site == 1) ? g_sid2 : g_sid;
    String &user = (site == 1) ? g_dsmUser2 : g_dsmUser;
    String &pass = (site == 1) ? g_dsmPassword2 : g_dsmPassword;
    String url = nasApiUrl(site)
        + "auth.cgi?api=SYNO.API.Auth&version=6&method=login"
        + "&account=" + user
        + "&passwd="  + pass
        + "&session=SurveillanceStation&format=sid";
    HTTPClient http;
    http.begin(url);
    http.setTimeout(6000);
    int code = http.GET();
    if (code != 200) { http.end(); return false; }
    String body = http.getString();
    http.end();
    JsonDocument doc;
    if (deserializeJson(doc, body) || !doc["success"].as<bool>()) return false;
    sid = doc["data"]["sid"].as<String>();
    Serial.printf("[SS] site%d sid=%s\n", site, sid.c_str());
    return true;
}

bool ssListCameras();  // defined below; called here so the very first
                       // snapshot already has a real name to overlay instead
                       // of falling back to "Cam N"

// camId here is the *combined* id described above g_camIds — decode back
// into (site, Synology-native id) before talking to either NAS.
size_t ssGetSnapshot(int combinedCamId) {
    int site  = (combinedCamId >> 16) & 1;
    int camId = combinedCamId & 0xFFFF;
    String &sid = (site == 1) ? g_sid2 : g_sid;
    if (sid.isEmpty() && !ssLogin(site)) return 0;
    if (g_camCount == 0) ssListCameras();
    for (int attempt = 0; attempt < 2; attempt++) {
        // Per Synology's official Web API docs, GetSnapshot's profileType param
        // (0=High quality, 1=Balanced, 2=Low bandwidth) is only defined for
        // version=9, and uses "id" — not "cameraId". We were on version=8 with
        // cameraId, an older calling convention that returns an image but
        // silently ignores profileType, which is why the earlier attempt had
        // no effect.
        String url = nasApiUrl(site)
            + "entry.cgi?api=SYNO.SurveillanceStation.Camera&version=9"
            + "&method=GetSnapshot&id=" + String(camId)
            + "&profileType=1"
            + "&_sid=" + sid;
        unsigned long tBegin = millis();
        HTTPClient http;
        http.begin(url);
        http.setTimeout(10000);
        unsigned long tGetStart = millis();
        int code = http.GET();
        unsigned long tGetMs = millis() - tGetStart;
        if (code != 200) {
            http.end(); sid = "";
            if (attempt == 0 && ssLogin(site)) continue;
            return 0;
        }
        int contentLen = http.getSize();  // -1 if server didn't send Content-Length
        WiFiClient *stream = http.getStreamPtr();
        stream->setTimeout(8000);
        // readBytes(buf, len) doesn't stop just because the response ended —
        // it keeps polling until no new bytes arrive for the full timeout,
        // then gives up. That was costing ~8s per fetch regardless of file
        // size (measured: readBytes took ~8000-8250ms whether the file was
        // 51KB or 140KB). Since Content-Length tells us the exact size,
        // reading exactly that many bytes lets it return as soon as the
        // last byte arrives instead of waiting out the timeout.
        size_t readLen = (contentLen > 0 && (size_t)contentLen <= MAX_JPEG_BYTES)
            ? (size_t)contentLen : MAX_JPEG_BYTES;
        unsigned long tReadStart = millis();
        size_t got = stream->readBytes(jpegBuf, readLen);
        unsigned long tReadMs = millis() - tReadStart;
        http.end();
        Serial.printf("[SS-TIMING] begin=%lums GET=%lums readBytes=%lums\n",
            tGetStart - tBegin, tGetMs, tReadMs);
        if (got < 4 || jpegBuf[0] != 0xFF || jpegBuf[1] != 0xD8) {
            sid = ""; return 0;
        }
        bool hasEOI = (got >= 2 && jpegBuf[got - 2] == 0xFF && jpegBuf[got - 1] == 0xD9);
        Serial.printf("[SS] %zu bytes site%d cam%d (Content-Length=%d, match=%s, EOI=%s)\n",
            got, site, camId, contentLen,
            (contentLen < 0 || (size_t)contentLen == got) ? "yes" : "NO-MISMATCH",
            hasEOI ? "yes" : "MISSING");
        return got;
    }
    return 0;
}

// Queries every configured NAS (site 0, plus site 1 when site2Configured())
// for every camera on that Surveillance Station install and caches the
// combined result in g_camerasJson for the web UI to build its button grid
// from. Returns true if at least one site's list succeeded — a site 2
// outage shouldn't take site 0's already-working cameras down with it.
bool ssListCamerasForSite(int site, String &json, bool &first) {
    String &sid = (site == 1) ? g_sid2 : g_sid;
    if (sid.isEmpty() && !ssLogin(site)) return false;
    for (int attempt = 0; attempt < 2; attempt++) {
        String url = nasApiUrl(site)
            + "entry.cgi?api=SYNO.SurveillanceStation.Camera&version=9"
            + "&method=List&_sid=" + sid;
        HTTPClient http;
        http.begin(url);
        http.setTimeout(8000);
        int code = http.GET();
        if (code != 200) {
            http.end(); sid = "";
            if (attempt == 0 && ssLogin(site)) continue;
            return false;
        }
        String body = http.getString();
        http.end();
        JsonDocument doc;
        if (deserializeJson(doc, body) || !doc["success"].as<bool>()) {
            sid = "";
            if (attempt == 0 && ssLogin(site)) continue;
            return false;
        }
        JsonArray cams = doc["data"]["cameras"].as<JsonArray>();
        for (JsonObject cam : cams) {
            // Synology's List response carries the user-assigned camera name
            // in "newName" ("name" isn't present in the payload at all) —
            // confirmed against a live NAS via serial debug logging.
            int id = cam["id"].as<int>();
            String name = cam["newName"].as<String>();
            int combinedId = (site << 16) | (id & 0xFFFF);
            if (g_camCount < MAX_CAMERAS) {
                g_camIds[g_camCount] = combinedId;
                g_camNames[g_camCount] = name;
                g_camCount++;
            }
            if (!first) json += ",";
            first = false;
            name.replace("\\", "\\\\");
            name.replace("\"", "\\\"");
            json += "{\"id\":" + String(combinedId)
                  + ",\"name\":\"" + name + "\"}";
        }
        Serial.printf("[SS] site%d listed %d cameras\n", site, cams.size());
        return true;
    }
    return false;
}

bool ssListCameras() {
    g_camCount = 0;
    String json = "[";
    bool first = true;
    bool ok0 = ssListCamerasForSite(0, json, first);
    bool ok1 = site2Configured() ? ssListCamerasForSite(1, json, first) : false;
    json += "]";
    g_camerasJson = json;
    return ok0 || ok1;
}

void drawError(const String &msg) {
    gfx->fillScreen(COL_BLACK);
    gfx->setTextColor(COL_RED);
    gfx->setTextSize(2);
    gfx->setCursor(20, 220);
    gfx->print(msg);
    presentG_fb();
}

void enterScreensaver() {
    gfx->fillScreen(COL_BLACK);
    presentG_fb();
    digitalWrite(LCD_BL, LOW);
}

void exitScreensaver() {
    digitalWrite(LCD_BL, HIGH);
}

// FT6336U TD_STATUS register (0x02): low nibble is the number of active
// touch points, followed immediately by touch point 1's X/Y (registers
// 0x03-0x06: XH, XL, YH, YL — top nibble of XH/YH is flags, not part of the
// coordinate). Reading it directly rather than pulling in the full FT6336U
// library — that library's begin() calls Wire.begin() again, which we don't
// need since the IO expander already initialized this bus.
//
// Confirmed on real hardware: taps at the bottom of the screen (near the
// arrow row) read back as raw y≈36-45 — i.e. the touch panel's Y axis runs
// opposite to the display's, same class of orientation mismatch already
// seen on this board's color channels. Y gets flipped here to correct that.
//
// X was assumed to match the display as-is based on an earlier ad-hoc test,
// but a deliberate bottom-right-then-bottom-left corner test told a
// different story: bottom-right read x=6 (far left) and bottom-left read
// x=466 (far right) — X is mirrored too. Flipped here as well.
bool readTouch(int &x, int &y) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(TOUCH_I2C_ADDR, 5) != 5) return false;
    uint8_t tdStatus = Wire.read();
    uint8_t xh = Wire.read();
    uint8_t xl = Wire.read();
    uint8_t yh = Wire.read();
    uint8_t yl = Wire.read();
    if ((tdStatus & 0x0F) == 0) return false;
    x = SCREEN_W - 1 - (((xh & 0x0F) << 8) | xl);
    y = SCREEN_H - 1 - (((yh & 0x0F) << 8) | yl);
    Serial.printf("[TOUCH] x=%d y=%d\n", x, y);
    return true;
}

void renderJpeg(size_t jpegLen) {
    if (!decodeBuf) { drawError("No decode buf"); return; }

#if USE_TJPG_DECODER
    uint16_t fw16, fh16;
    if (TJpgDec.getJpgSize(&fw16, &fh16, jpegBuf, jpegLen) != JDR_OK) {
        drawError("TJpg getSize failed"); return;
    }
    int fw = fw16, fh = fh16;
    // Pick the largest power-of-2 downscale (least decode work) that still
    // keeps decodedW >= SCREEN_W — blitFit shrinks to fit the screen anyway,
    // so decoding above that resolution just burns CPU for no visible gain.
    // Tried dropping to SCREEN_W/2 (scale=4, 320x180) for more speed — only
    // saved ~100-150ms out of ~700ms decode (most of the cost is Huffman/
    // entropy decoding the full bitstream, which doesn't shrink with output
    // scale) but made foliage/texture noticeably blurrier on the real
    // screen. Not a good trade — reverted to SCREEN_W.
    int scale = 1;
    while (scale < 8 && (fw / (scale * 2)) >= SCREEN_W) scale *= 2;
    decodedW = fw / scale; decodedH = fh / scale;
    while ((size_t)(decodedW * decodedH * 2) > MAX_DECODE_BYTES && scale < 8) {
        scale *= 2;
        decodedW = fw / scale; decodedH = fh / scale;
    }
    TJpgDec.setJpgScale(scale);
    Serial.printf("[RENDER-TJPG] %dx%d → %dx%d\n", fw, fh, decodedW, decodedH);
    unsigned long tDecodeStart = millis();
    TJpgDec.drawJpg(0, 0, jpegBuf, jpegLen);
    Serial.printf("[RENDER-TIMING] decode=%lums\n", millis() - tDecodeStart);
#else
    if (!jpeg.openRAM(jpegBuf, (int)jpegLen, jpegDrawCB)) {
        drawError("JPEG open failed"); return;
    }

    // Prefer full resolution (no IDCT scaling at all) now that the camera
    // stream is small enough (1280x720) to fit — isolates whether
    // JPEGDEC's scaled-decode path was ever part of the corruption we've
    // been chasing. Falls back to half/quarter only if it doesn't fit.
    int fw = jpeg.getWidth(), fh = jpeg.getHeight();
    int scale = 0;  // JPEGDEC has no "NONE" constant — 0 means full resolution, no IDCT scaling
    decodedW = fw;  decodedH = fh;
    if ((size_t)(decodedW * decodedH * 2) > MAX_DECODE_BYTES) {
        scale = JPEG_SCALE_HALF;
        decodedW = fw / 2;  decodedH = fh / 2;
        if ((size_t)(decodedW * decodedH * 2) > MAX_DECODE_BYTES) {
            scale = JPEG_SCALE_QUARTER;
            decodedW = fw / 4;  decodedH = fh / 4;
        }
    }
    Serial.printf("[RENDER] %dx%d → %dx%d\n", fw, fh, decodedW, decodedH);

    jpeg.setPixelType(RGB565_BIG_ENDIAN);
    dbgCbCount = dbgMaxX = dbgMaxY = dbgMaxIW = dbgMaxIH = 0;
    jpeg.decode(0, 0, scale);
    jpeg.close();
    Serial.printf("[DEBUG] callbacks=%d maxX=%d maxY=%d maxIW=%d maxIH=%d (decodedW=%d decodedH=%d)\n",
        dbgCbCount, dbgMaxX, dbgMaxY, dbgMaxIW, dbgMaxIH, decodedW, decodedH);
#endif

    // decodeBuf is PSRAM, written via many small scattered callback writes
    // during decode (unlike the single bulk write in the synthetic
    // gradient test, which rendered perfectly) — flush the cache before
    // blitWindow reads it, in case some tiles' writes aren't yet visible.
    unsigned long tFlushStart = millis();
    Cache_WriteBack_Addr((uint32_t)decodeBuf, (size_t)decodedW * decodedH * 2);
    unsigned long tFlushMs = millis() - tFlushStart;

    unsigned long tBlitStart = millis();
    blitFit();
    Serial.printf("[RENDER-TIMING] cacheFlush=%lums blit=%lums\n", tFlushMs, millis() - tBlitStart);
}

WebServer webServer(80);

const char* PAGE_HTML = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<title>SenseCAP Camera Viewer</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:sans-serif;max-width:600px;margin:20px auto;padding:0 10px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:10px 0}
button{padding:12px 8px;background:#222;color:#fff;border:none;border-radius:6px;cursor:pointer;font-size:.95em;width:100%}
button:active{background:#444}
.status{background:#f0f0f0;padding:8px;border-radius:4px;margin:10px 0;font-size:.9em}
#img{display:none;max-width:100%;border-radius:6px;margin-top:12px}
.links{text-align:center}
</style>
</head>
<body>
<h1>Cameras</h1>
<div class="status" id="status">Loading...</div>
<div class="grid" id="grid"></div>
<img id="img" alt="snapshot">
<p class="links"><a href="/settings">Settings</a> &nbsp;|&nbsp; <a href="#" onclick="loadCameras();return false;">Refresh Camera List</a> &nbsp;|&nbsp; <a href="#" onclick="reconfigure();return false;">Reset Device</a></p>
<script>
function reconfigure(){
  if(!confirm('This clears the saved WiFi and NAS settings and reboots into setup mode. Continue?')) return;
  fetch('/reconfigure').then(function(){
    document.getElementById('status').innerText='Rebooting into setup mode...';
  });
}
var camNames={};
function loadCameras(force){
  var grid=document.getElementById('grid');
  grid.innerHTML='Loading cameras...';
  fetch('/cameras'+(force?'?refresh=1':'')).then(r=>r.json()).then(function(cams){
    camNames={};
    grid.innerHTML='';
    if(!cams.length){
      grid.innerHTML='<p>No cameras found. Check Settings.</p>';
      return;
    }
    cams.forEach(function(c){
      camNames[c.id]=c.name;
      var b=document.createElement('button');
      b.textContent=c.name;
      b.onclick=function(){snap(c.id,c.name);};
      grid.appendChild(b);
    });
  });
}
function updateStatus(){fetch('/status').then(r=>r.json()).then(d=>{
  var name=camNames[d.lastCam]||('Cam '+d.lastCam);
  document.getElementById('status').innerText='State: '+d.state+(d.lastCam?(' | Last: '+name):'');
});}
function snap(cam,name){
  document.getElementById('status').innerText='Fetching '+name+'...';
  fetch('/trigger?cam='+cam).then(function(){
    // Deadline, not an attempt count: the ESP32's web server is single-
    // threaded and can't answer /status at all while a fetch is in flight
    // (see busy below), so a slow poll response eats into the wait just as
    // much as the interval between polls does. NAS fetches have been
    // observed taking up to ~36s under load, so give this comfortable
    // margin above that rather than the old fixed 10s/20-attempt cutoff,
    // which was cutting off well before slow-but-real fetches finished and
    // leaving the page showing a stale or missing snapshot.
    var deadline=Date.now()+60000;
    var poll=setInterval(function(){
      fetch('/status').then(r=>r.json()).then(function(d){
        if(!d.busy||Date.now()>deadline){
          clearInterval(poll);
          var img=document.getElementById('img');
          img.onload=function(){img.style.display='block';};
          img.src='/snapshot?t='+Date.now();
          var name2=camNames[d.lastCam]||('Cam '+d.lastCam);
          document.getElementById('status').innerText='State: '+d.state+(d.lastCam?' | Last: '+name2:'');
        }
      });
    },500);
  });
}
loadCameras();
updateStatus(); setInterval(updateStatus,5000);
</script>
</body>
</html>
)rawhtml";

// NAS settings page — separate from the WiFiManager captive portal, which
// only appears when WiFi credentials are missing/invalid. This is always
// reachable from the normal web UI so NAS host/port/user/password can be
// changed without forcing a WiFi reconfigure. Password field is left blank
// on load (never echoes the stored value); submitting it blank keeps the
// existing password unchanged.
String settingsPageHtml(const String &msg = "") {
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<title>Settings</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:0 10px}
label{display:block;margin-top:12px;font-size:.9em;color:#444}
input,select{width:100%;padding:8px;font-size:1em;box-sizing:border-box;margin-top:4px}
input[type=checkbox],input[type=radio]{width:auto;margin-top:0;margin-right:6px}
.check-row{display:flex;align-items:center;margin-top:12px;font-size:.9em;color:#444}
.radio-row{display:flex;align-items:center;gap:16px;margin-top:6px;font-size:.9em;color:#444}
.radio-row label{display:flex;align-items:center;margin-top:0;font-size:1em;color:inherit}
h2{font-size:1.05em;margin:24px 0 0}
button{margin-top:18px;padding:10px 16px;background:#222;color:#fff;border:none;border-radius:6px;font-size:1em}
.msg{background:#e6ffe6;padding:8px;border-radius:4px;margin:10px 0;font-size:.9em}
</style>
</head>
<body>
<h1>Settings</h1>
)rawhtml";
    if (msg.length()) html += "<div class=\"msg\">" + msg + "</div>";
    html += "<form method=\"POST\" action=\"/settings\">";
    html += "<label>NAS Host/IP<input name=\"nasHost\" value=\"" + g_nasHost + "\"></label>";
    html += "<label>NAS Port<input name=\"dsmPort\" value=\"" + String(g_dsmPort) + "\"></label>";
    html += "<label>NAS Username<input name=\"dsmUser\" value=\"" + g_dsmUser + "\"></label>";
    html += "<label>NAS Password (leave blank to keep current)<input type=\"password\" name=\"dsmPass\" value=\"\"></label>";

    html += "<h2>Second NAS (optional — e.g. a remote site reached over Tailscale)</h2>";
    html += "<label>NAS Host/IP<input name=\"nasHost2\" value=\"" + g_nasHost2 + "\" placeholder=\"leave blank to disable\"></label>";
    html += "<label>NAS Port<input name=\"dsmPort2\" value=\"" + String(g_dsmPort2) + "\"></label>";
    html += "<label>NAS Username<input name=\"dsmUser2\" value=\"" + g_dsmUser2 + "\"></label>";
    html += "<label>NAS Password (leave blank to keep current)<input type=\"password\" name=\"dsmPass2\" value=\"\"></label>";

    html += "<h2>Display Overlay</h2>";
    html += "<div class=\"check-row\"><input type=\"checkbox\" id=\"showCam\" name=\"showCam\""
          + String(g_showCamName ? " checked" : "") + "><label for=\"showCam\">Show camera name</label></div>";
    html += "<div class=\"check-row\"><input type=\"checkbox\" id=\"showTime\" name=\"showTime\""
          + String(g_showDateTime ? " checked" : "") + "><label for=\"showTime\">Show date/time</label></div>";

    html += "<label>Clock format</label><div class=\"radio-row\">"
            "<label><input type=\"radio\" name=\"clockFmt\" value=\"24\"" + String(g_use24Hour ? " checked" : "") + "> 24-hour</label>"
            "<label><input type=\"radio\" name=\"clockFmt\" value=\"12\"" + String(!g_use24Hour ? " checked" : "") + "> 12-hour</label>"
            "</div>";

    html += "<label>Date order<select name=\"dateFmt\">";
    html += "<option value=\"ymd\"" + String(g_dateFormat == "ymd" ? " selected" : "") + ">YYYY-MM-DD</option>";
    html += "<option value=\"mdy\"" + String(g_dateFormat == "mdy" ? " selected" : "") + ">MM-DD-YYYY</option>";
    html += "<option value=\"dmy\"" + String(g_dateFormat == "dmy" ? " selected" : "") + ">DD-MM-YYYY</option>";
    html += "</select></label>";

    html += "<button type=\"submit\">Save</button></form>";
    html += "<p><a href=\"/\">Back</a></p></body></html>";
    return html;
}

enum State { S_SCREENSAVER, S_IMAGE, S_ERROR };
State         appState     = S_SCREENSAVER;
unsigned long lastActivity = 0;
volatile bool pendingMotion = false;
volatile int  pendingCamId  = 0;
bool          processing    = false;
unsigned long debounceMs    = DEBOUNCE_MS;
unsigned long saverMs       = SCREENSAVER_TIMEOUT_MS;

const char* stateName() {
    switch(appState) {
        case S_SCREENSAVER: return "screensaver";
        case S_IMAGE:       return "image";
        default:            return "error";
    }
}

void setupWebServer() {
    webServer.on("/", HTTP_GET, [](){
        webServer.send(200, "text/html", PAGE_HTML);
    });
    webServer.on("/status", HTTP_GET, [](){
        // busy covers both an in-flight fetch (processing) and a trigger that's
        // been queued but not yet picked up by loop() (pendingMotion) — without
        // the latter, a poll landing in that narrow gap would see processing
        // still false and wrongly conclude the request that was just submitted
        // had already finished.
        String json = "{\"state\":\"" + String(stateName()) + "\""
            + ",\"lastCam\":" + String(lastCamId)
            + ",\"busy\":" + (processing || pendingMotion ? "true" : "false")
            + "}";
        webServer.send(200, "application/json", json);
    });
    webServer.on("/cameras", HTTP_GET, [](){
        if (g_camerasJson.isEmpty() || webServer.hasArg("refresh")) ssListCameras();
        webServer.send(200, "application/json", g_camerasJson.isEmpty() ? "[]" : g_camerasJson);
    });
    webServer.on("/snapshot", HTTP_GET, [](){
        if (snapLen > 0)
            webServer.send_P(200, "image/jpeg", (const char*)snapBuf, snapLen);
        else
            webServer.send(404, "text/plain", "no snapshot yet");
    });
    webServer.on("/trigger", HTTP_GET, [](){
        String camStr = webServer.arg("cam");
        if (camStr.isEmpty()) { webServer.send(400, "text/plain", "missing cam"); return; }
        int camId = parseCamParam(camStr);
        // A name lookup can miss because the list has never been fetched, or
        // because a stale cached name no longer matches — refresh once and
        // retry before giving up either way. Numeric ids never hit this path.
        if (camId < 0) { ssListCameras(); camId = parseCamParam(camStr); }
        if (camId < 0) {
            Serial.printf("[TRIGGER] unknown cam, raw value: \"%s\"\n", camStr.c_str());
            webServer.send(400, "text/plain", "unknown cam");
            return;
        }
        webServer.send(200, "text/plain", "OK");
        pendingCamId  = camId;
        pendingMotion = true;
    });
    webServer.on("/motion", HTTP_GET, [](){
        String camStr = webServer.arg("cam");
        if (camStr.isEmpty()) { webServer.send(400, "text/plain", "missing cam"); return; }
        int camId = parseCamParam(camStr);
        if (camId < 0) { ssListCameras(); camId = parseCamParam(camStr); }
        if (camId < 0) {
            Serial.printf("[MOTION] unknown cam, raw value: \"%s\"\n", camStr.c_str());
            webServer.send(400, "text/plain", "unknown cam");
            return;
        }
        webServer.send(200, "text/plain", "OK");
        pendingCamId  = camId;
        pendingMotion = true;
    });
    webServer.on("/reboot", HTTP_GET, [](){
        webServer.send(200, "text/plain", "Rebooting...");
        delay(300);
        ESP.restart();
    });
    webServer.on("/reconfigure", HTTP_GET, [](){
        webServer.send(200, "text/plain", "Clearing WiFi + NAS config, rebooting...");
        delay(300);
        WiFiManager wm;
        wm.resetSettings();
        g_configPrefs.begin("sitecfg", false);
        g_configPrefs.clear();
        g_configPrefs.end();
        delay(200);
        ESP.restart();
    });
    webServer.on("/settings", HTTP_GET, [](){
        webServer.send(200, "text/html", settingsPageHtml());
    });
    webServer.on("/settings", HTTP_POST, [](){
        g_nasHost = webServer.arg("nasHost");
        g_dsmPort = webServer.arg("dsmPort").toInt();
        g_dsmUser = webServer.arg("dsmUser");
        String newPass = webServer.arg("dsmPass");
        if (newPass.length()) g_dsmPassword = newPass;

        g_nasHost2 = webServer.arg("nasHost2");
        g_dsmPort2 = webServer.arg("dsmPort2").toInt();
        g_dsmUser2 = webServer.arg("dsmUser2");
        String newPass2 = webServer.arg("dsmPass2");
        if (newPass2.length()) g_dsmPassword2 = newPass2;

        saveSiteConfig();
        g_sid = "";   // force re-login with the new credentials
        g_sid2 = "";
        g_camerasJson = "";  // force a camera list refresh against the new NAS/credentials

        // Checkboxes are only present in the POST body when checked.
        g_showCamName  = webServer.hasArg("showCam");
        g_showDateTime = webServer.hasArg("showTime");
        g_use24Hour    = webServer.arg("clockFmt") != "12";
        String dateFmt = webServer.arg("dateFmt");
        if (dateFmt == "mdy" || dateFmt == "dmy" || dateFmt == "ymd") g_dateFormat = dateFmt;
        saveDisplayConfig();

        webServer.send(200, "text/html", settingsPageHtml("Saved. New settings will be used on the next fetch."));
    });
    webServer.begin();
    Serial.println("[BOOT] Web server on :80");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[BOOT] SenseCAP Camera Viewer (PlatformIO)");

    loadSiteConfig();
    loadDisplayConfig();

    // Tested dropping WiFi TX power to minimum (WIFI_POWER_MINUS_1dBm) to
    // check for RF coupling into the display bus — made no difference, the
    // same shifting/splitting still occurred, ruling that out. Reverted to
    // normal power so WiFi range/reliability isn't needlessly degraded.
    WiFi.mode(WIFI_STA);

#if USE_TJPG_DECODER
    TJpgDec.setCallback(tjpgOutputCB);
    TJpgDec.setSwapBytes(false);  // matches remap_pixel's native-order bit extraction
#endif

    Wire.begin(I2C_SDA, I2C_SCL, 400000);
    delay(50);

    ioex.attach(Wire);
    ioex.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
    ioex.direction(PCA95x5::Direction::OUT_ALL);

    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    ioex.write(PCA95x5::Port::P05, PCA95x5::Level::L);
    delay(500);
    ioex.write(PCA95x5::Port::P05, PCA95x5::Level::H);
    delay(1000);
    // CS (P04) is now toggled automatically per-transaction by
    // Arduino_SWSPI_PCA9535CS::beginWrite()/endWrite() — no manual assert
    // needed here, and holding it low for the whole init burst (the old
    // approach) was likely desyncing the bit-banged command framing.
    ioex.write(PCA95x5::Port::P04, PCA95x5::Level::H);

    bool ok = gfx->begin();
    // NOTE: no gfx->setRotation() here — a leftover call to setRotation(2)
    // used to live on this line, predating presentG_fb()'s manual 180° flip
    // (added when text was found upside-down). Having BOTH active at once
    // meant two independent rotation transforms were stacking, which is
    // what produced split/scrambled text on the AP-setup screen. The
    // manual flip in presentG_fb() is now the only rotation mechanism.
    Serial.printf("[BOOT] Display: %s\n", ok ? "OK" : "FAIL");

    g_fb = gfx->getFramebuffer();
    Serial.printf("[BOOT] Framebuffer: %s\n", g_fb ? "OK" : "NULL");

    if (!g_fb) { Serial.println("Framebuffer null — halting"); while(1) delay(5000); }

    if (!rgbpanel->getFrameBuffers(SCREEN_W, SCREEN_H, &fbBuf[0], &fbBuf[1])) {
        Serial.println("getFrameBuffers() failed — halting"); while(1) delay(5000);
    }
    Serial.printf("[BOOT] Double buffers: fb0=%p fb1=%p\n", fbBuf[0], fbBuf[1]);

#if COLOR_LANE_TEST
    // Tests remap_pixel() itself (not raw fillScreen) so this actually
    // validates the rotation-fix math, including mixed-channel colors that
    // a real photo can't isolate cleanly.
    Serial.println("[TEST] remap_pixel() test — intended colors, 3s each, loops forever");
    uint16_t trueCols[]    = { 0xF800, 0x07E0, 0x001F, 0x0000, 0xFFFF, 0x8410, 0xFFE0, 0x07FF, 0xF81F };
    const char* colNames[] = { "RED",   "GREEN", "BLUE",  "BLACK", "WHITE", "GRAY",  "YELLOW","CYAN", "MAGENTA" };
    while (true) {
        for (int i = 0; i < 9; i++) {
            uint16_t out = remap_pixel(trueCols[i]);
            gfx->fillScreen(out);
            presentG_fb();
            Serial.printf("[TEST] intend=0x%04X sent=0x%04X  %s\n", trueCols[i], out, colNames[i]);
            delay(3000);
        }
    }
#endif

    gfx->fillScreen(COL_BLACK);
    gfx->setTextColor(COL_GREEN);
    gfx->setTextSize(2);
    gfx->setCursor(80, 215);
    gfx->print("Connecting to WiFi...");
    presentG_fb();

    // WiFiManager: tries the previously-connected network first (stored by
    // the ESP32 WiFi stack itself, not by us); if that fails it opens its
    // own "SenseCAP-Setup" access point with a captive-portal config page
    // for entering new WiFi credentials plus the NAS host/port/user/
    // password fields below. No hardcoded WiFi/NAS values needed anymore —
    // see loadSiteConfig()/saveSiteConfig() for where these persist.
    WiFiManager wm;
    wm.setAPCallback([](WiFiManager *mgr) {
        gfx->fillScreen(COL_BLACK);
        gfx->setTextColor(COL_GREEN);
        gfx->setTextSize(2);
        gfx->setCursor(10, 170);
        gfx->print("Setup needed");
        gfx->setCursor(10, 210);
        gfx->print("Join WiFi:");
        gfx->setCursor(10, 235);
        gfx->print(mgr->getConfigPortalSSID().c_str());
        gfx->setCursor(10, 275);
        gfx->print("Then open 192.168.4.1");
        presentG_fb();
    });

    String defNasHost = g_nasHost;
    String defDsmPort = String(g_dsmPort);
    String defDsmUser = g_dsmUser;
    String defDsmPass = g_dsmPassword;
    WiFiManagerParameter custom_nas_host("nasHost", "NAS Host/IP", defNasHost.c_str(), 40);
    WiFiManagerParameter custom_dsm_port("dsmPort", "NAS Port", defDsmPort.c_str(), 6);
    WiFiManagerParameter custom_dsm_user("dsmUser", "NAS Username", defDsmUser.c_str(), 40);
    WiFiManagerParameter custom_dsm_pass("dsmPass", "NAS Password", defDsmPass.c_str(), 40);
    wm.addParameter(&custom_nas_host);
    wm.addParameter(&custom_dsm_port);
    wm.addParameter(&custom_dsm_user);
    wm.addParameter(&custom_dsm_pass);

    // TEST: non-blocking config portal + our own wait loop instead of the
    // single blocking autoConnect() call, so we can periodically re-present
    // the (unchanged) setup screen while waiting. Working theory: a static
    // frame left on screen for a long, uninterrupted stretch (which the
    // setup screen is, while the user reads instructions/connects/submits
    // the form) accumulates a scanout/DMA glitch over time — the same
    // shifting/splitting appeared on regular snapshots too, which similarly
    // sit static for a while between triggers, so if periodic re-presenting
    // prevents it here, that's a mitigation for normal photo viewing too.
    wm.setConfigPortalBlocking(false);
    bool wmConnected = wm.autoConnect("SenseCAP-Setup");
    unsigned long lastKeepAlive = millis();
    while (!wmConnected) {
        wm.process();
        wmConnected = (WiFi.status() == WL_CONNECTED);
        if (millis() - lastKeepAlive > 2000) {
            Cache_WriteBack_Addr((uint32_t)fbBuf[0], SCREEN_W * SCREEN_H * 2);
            rgbpanel->presentBuffer(fbBuf[0], SCREEN_W, SCREEN_H);
            lastKeepAlive = millis();
        }
        delay(10);
    }
    // Persist whatever's in the fields now — either freshly submitted via
    // the portal, or unchanged if autoConnect succeeded without needing it.
    g_nasHost     = custom_nas_host.getValue();
    g_dsmPort     = atoi(custom_dsm_port.getValue());
    g_dsmUser     = custom_dsm_user.getValue();
    g_dsmPassword = custom_dsm_pass.getValue();
    saveSiteConfig();

    String ipStr = WiFi.localIP().toString();
    Serial.printf("\n[BOOT] IP: %s\n", ipStr.c_str());

    gfx->fillScreen(COL_BLACK);
    gfx->setTextColor(COL_GREEN);
    gfx->setTextSize(2);
    gfx->setCursor(60, 215);
    gfx->print("Connected!");
    gfx->setCursor(30, 245);
    gfx->print("IP: " + ipStr);
    presentG_fb();
    delay(4000);

    configTime(-7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("[BOOT] NTP started (background)");

    Serial.printf("[BOOT] psramFound=%d psramSize=%u freePs=%u\n",
        (int)psramFound(), ESP.getPsramSize(), ESP.getFreePsram());
    Serial.printf("[BOOT] spiram_cap=%u spiram8=%u\n",
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    // Try heap_caps_malloc directly in case ps_malloc differs in this IDF build
    jpegBuf = (uint8_t *)heap_caps_malloc(MAX_JPEG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jpegBuf) jpegBuf = (uint8_t *)malloc(MAX_JPEG_BYTES);  // fallback to heap
    Serial.printf("[BOOT] jpegBuf: %s @ %p\n", jpegBuf ? "OK" : "FAIL", jpegBuf);
    snapBuf = (uint8_t *)heap_caps_malloc(MAX_JPEG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!snapBuf) snapBuf = (uint8_t *)malloc(MAX_JPEG_BYTES);
    Serial.printf("[BOOT] snapBuf: %s @ %p\n", snapBuf ? "OK" : "FAIL", snapBuf);
    decodeBuf = (uint16_t *)heap_caps_malloc(MAX_DECODE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    Serial.printf("[BOOT] decodeBuf: %s @ %p\n", decodeBuf ? "OK" : "FAIL", decodeBuf);
    bool historyOk = true;
    for (int i = 0; i < HISTORY_SLOTS; i++) {
        g_historyBuf[i] = (uint16_t *)heap_caps_malloc(SCREEN_W * SCREEN_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        Serial.printf("[BOOT] historyBuf[%d]: %s @ %p\n", i, g_historyBuf[i] ? "OK" : "FAIL", g_historyBuf[i]);
        if (!g_historyBuf[i]) historyOk = false;
    }
    if (!jpegBuf || !snapBuf || !decodeBuf || !historyOk) {
        gfx->fillScreen(COL_BLACK);
        gfx->setTextColor(COL_GREEN);
        gfx->setTextSize(2);
        gfx->setCursor(20, 215);
        gfx->print("PSRAM alloc failed");
        presentG_fb();
        while (1) delay(5000);
    }
    Serial.println("[BOOT] PSRAM OK");

#if GRADIENT_TEST
    // Fine checkerboard + smooth gradient, generated entirely in code —
    // no JPEG, no camera, no network. If this shows the same swirly
    // corruption, the display hardware itself can't handle fine detail
    // regardless of what feeds it. If it's clean, the bug is somewhere in
    // the JPEG decode chain after all, despite every JPEGDEC-side test.
    decodedW = 480; decodedH = 480;
    for (int y = 0; y < decodedH; y++) {
        for (int x = 0; x < decodedW; x++) {
            uint16_t r = (x * 31) / decodedW;
            uint16_t g = (y * 63) / decodedH;
            uint16_t b = ((x + y) * 31) / (decodedW + decodedH);
            bool checker = ((x / 4) + (y / 4)) % 2;
            if (checker) { r = 31 - r; g = 63 - g; b = 31 - b; }
            decodeBuf[y * decodedW + x] = (r << 11) | (g << 5) | b;
        }
    }
    Serial.println("[TEST] Gradient test pattern generated, displaying...");
    exitScreensaver();
    while (true) {
        blitWindow(0, 0);
        delay(3000);
    }
#endif

    setupWebServer();
    Serial.println("[BOOT] Web server started");

    // ssLogin done lazily on first snapshot request
    enterScreensaver();
    lastActivity = millis();
    Serial.println("[BOOT] Ready");
    Serial.printf("[HEAP] free=%u minFree=%u freePsram=%u uptime=%lus\n",
        ESP.getFreeHeap(), ESP.getMinFreeHeap(),
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM), millis() / 1000);
}

void loop() {
    webServer.handleClient();

    if (pendingMotion && !processing) {
        int camId = pendingCamId;
        pendingMotion = false;

        unsigned long now = millis();
        if (appState == S_IMAGE && lastCamId == camId &&
            (now - lastActivity) < debounceMs) {
            Serial.printf("[DEBOUNCE] cam%d skipped\n", camId);
            return;
        }

        processing = true;
        lastActivity = now;
        lastCamId = camId;

        exitScreensaver();

        unsigned long tFetchStart = millis();
        size_t len = ssGetSnapshot(camId);
        unsigned long tFetchMs = millis() - tFetchStart;
        if (len > 0) {
            memcpy(snapBuf, jpegBuf, len);
            snapLen = len;
            snapCam = camId;
            snapTime = time(nullptr);
            unsigned long tRenderStart = millis();
            renderJpeg(len);
            unsigned long tRenderMs = millis() - tRenderStart;
            Serial.printf("[TIMING] fetch=%lums render=%lums total=%lums\n",
                tFetchMs, tRenderMs, tFetchMs + tRenderMs);
            Serial.printf("[HEAP] free=%u minFree=%u freePsram=%u uptime=%lus\n",
                ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM), millis() / 1000);
            appState = S_IMAGE;
        } else {
            drawError("Snapshot failed cam" + String(camId));
            appState = S_ERROR;
        }
        processing = false;
    }

    // Screensaver timeout — blank screen and kill backlight
    if (appState == S_IMAGE &&
        (millis() - lastActivity) > saverMs) {
        enterScreensaver();
        appState = S_SCREENSAVER;
    }

    // Touch handling: tap anywhere to wake from screensaver (shows the most
    // recent history frame directly — no NAS fetch, no JPEG decode). While
    // awake, tapping the arrow zones (same row as the date/time bar, pinned
    // to the screen edges — see g_barCenterY, set each render in blitFit())
    // pages through history. Edge-triggered on the leading edge of a touch
    // (not "while held") so one tap doesn't skip through several frames.
    //
    // DIAGNOSTIC: polled at 100ms originally; the touch controller and the
    // display's chip-select line share the same I2C bus, and this display
    // has a long history of timing-sensitive scanout corruption (see
    // remap_pixel/double-buffering comments elsewhere in this file). 500ms
    // seemed to clear up the corruption but was slow enough to miss short
    // taps entirely (a normal tap is ~100-300ms, so a 500ms poll gap has a
    // real chance of sampling zero times during the touch). 150ms is the
    // compromise — still a real reduction from 100ms, but comfortably
    // faster than a typical tap duration so it shouldn't get missed.
    static unsigned long lastTouchPollMs = 0;
    static bool wasTouching = false;
    const int ARROW_ZONE_W = 100;
    const int ARROW_ZONE_HALF_H = 50;
    if (millis() - lastTouchPollMs > 150) {
        lastTouchPollMs = millis();
        int tx = 0, ty = 0;
        bool touching = readTouch(tx, ty);
        if (touching && !wasTouching) {
            lastActivity = millis();
            if (appState == S_SCREENSAVER) {
                if (g_historyCount > 0) {
                    exitScreensaver();
                    showHistoryOffset(0);
                    appState = S_IMAGE;
                }
            } else if (appState == S_IMAGE) {
                bool inArrowRow = ty > g_barCenterY - ARROW_ZONE_HALF_H &&
                                   ty < g_barCenterY + ARROW_ZONE_HALF_H;
                bool canGoOlder = g_historyOffset < g_historyCount - 1;
                bool canGoNewer = g_historyOffset > 0;
                if (inArrowRow && tx < ARROW_ZONE_W && canGoOlder) {
                    Serial.printf("[TOUCH] left arrow hit (offset %d->%d)\n", g_historyOffset, g_historyOffset + 1);
                    showHistoryOffset(g_historyOffset + 1);  // older
                } else if (inArrowRow && tx > SCREEN_W - ARROW_ZONE_W && canGoNewer) {
                    Serial.printf("[TOUCH] right arrow hit (offset %d->%d)\n", g_historyOffset, g_historyOffset - 1);
                    showHistoryOffset(g_historyOffset - 1);  // newer
                }
            }
        }
        wasTouching = touching;
    }

    // Keep-alive re-present: TEST for whether a long-uninterrupted static
    // frame accumulates a scanout/DMA glitch over time (matches what was
    // seen on the setup screen — correct at first, then drifting/splitting
    // the longer it sat unchanged). A photo can sit displayed for minutes
    // between motion triggers, so if this mitigates it, it matters for
    // normal viewing, not just the rare setup flow.
    static unsigned long lastKeepAliveMs = 0;
    if (appState == S_IMAGE && millis() - lastKeepAliveMs > 2000) {
        Cache_WriteBack_Addr((uint32_t)fbBuf[activeBuf], SCREEN_W * SCREEN_H * 2);
        rgbpanel->presentBuffer(fbBuf[activeBuf], SCREEN_W, SCREEN_H);
        lastKeepAliveMs = millis();
    }
}
