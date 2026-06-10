/**
 * screenView.cpp - CPAP menu view (rendered entirely in user code).
 */
#include <gui/screen_screen/screenView.hpp>
#include <gui/cpap/cpap_menu.hpp>
#include <gui/cpap/cpap_settings.hpp>

#include <texts/TextKeysAndLanguages.hpp>
#include <touchgfx/Color.hpp>
#include <touchgfx/Unicode.hpp>

#include <string.h>
#include <stdio.h>

extern "C" {
    #include "rotary_input.h"
    #include "dbg_log.h"
}

using namespace touchgfx;

namespace {

/* Palette (16bpp RGB565). Kept inline so a future theme swap is one place. */
const colortype kColBg        = Color::getColorFromRGB( 12,  18,  32); /* near-black blue   */
const colortype kColStatusBg  = Color::getColorFromRGB( 28,  40,  64);
const colortype kColTitleBg   = Color::getColorFromRGB( 18,  28,  48);
const colortype kColContentBg = Color::getColorFromRGB( 18,  28,  48);
const colortype kColRowSel    = Color::getColorFromRGB( 60, 140, 200); /* teal highlight    */
const colortype kColRowEdit   = Color::getColorFromRGB(200, 150,  50); /* amber for edit    */
const colortype kColText      = Color::getColorFromRGB(235, 240, 248);
const colortype kColTextDim   = Color::getColorFromRGB(160, 175, 200);
const colortype kColWave      = Color::getColorFromRGB(120, 200, 255);

constexpr int16_t kScreenW   = 480;
constexpr int16_t kScreenH   = 272;
constexpr int16_t kStatusH   = 32;
constexpr int16_t kTitleY    = 32;
constexpr int16_t kTitleH    = 38;
constexpr int16_t kContentY  = 70;
constexpr int16_t kContentH  = kScreenH - kContentY;
constexpr int16_t kRowH      = 28;
constexpr int16_t kRowsTopY  = kContentY + 4;

/* Copy an ASCII C string into a UnicodeChar buffer, clamped to buf_chars-1
 * plus a NUL terminator. */
void utf8ToBuf(const char* src, Unicode::UnicodeChar* dst, uint16_t buf_chars)
{
    if (src == nullptr) src = "";
    Unicode::strncpy(dst, src, buf_chars - 1U);
    dst[buf_chars - 1U] = 0;
}

} // namespace

screenView::screenView()
    : row_visible_count_(0U),
      list_scroll_offset_(0U)
{
}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();
    buildWidgets();
    renderCurrentPage();
    /* Note: Screen::handleTickEvent() is invoked automatically by the
     * Application on the active screen each frame, no registerTimerWidget()
     * needed (Screen is not a Drawable, so that API does not apply here). */
}

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();
}

void screenView::buildWidgets()
{
    /* Hide whatever the Designer base put on the screen (the puzzle image
     * and the test button). We add our own children below. */
    image1.setVisible(false);
    buttonWithLabel1.setVisible(false);
    /* The Designer base sized __background as 479x271 (off-by-one). Resize
     * it so the chosen colour fills the full panel. */
    __background.setPosition(0, 0, kScreenW, kScreenH);
    __background.setColor(kColBg);

    /* --------- Status bar (always visible) ------------------------- */
    status_bar_bg_.setPosition(0, 0, kScreenW, kStatusH);
    status_bar_bg_.setColor(kColStatusBg);
    add(status_bar_bg_);

    status_left_text_.setPosition(8, 6, 220, kStatusH - 12);
    status_left_text_.setColor(kColText);
    status_left_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("Home", status_left_buf_, sizeof(status_left_buf_) / sizeof(status_left_buf_[0]));
    status_left_text_.setWildcard(status_left_buf_);
    add(status_left_text_);

    status_right_text_.setPosition(kScreenW - 252, 6, 244, kStatusH - 12);
    status_right_text_.setColor(kColTextDim);
    status_right_text_.setTypedText(TypedText(T_DYN_M));   /* right-aligned via spaces */
    utf8ToBuf("", status_right_buf_, sizeof(status_right_buf_) / sizeof(status_right_buf_[0]));
    status_right_text_.setWildcard(status_right_buf_);
    add(status_right_text_);

    /* --------- Title bar ------------------------------------------- */
    title_bar_bg_.setPosition(0, kTitleY, kScreenW, kTitleH);
    title_bar_bg_.setColor(kColTitleBg);
    add(title_bar_bg_);

    title_text_.setPosition(16, kTitleY + 4, kScreenW - 32, kTitleH - 8);
    title_text_.setColor(kColText);
    title_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("CPAP", title_buf_, sizeof(title_buf_) / sizeof(title_buf_[0]));
    title_text_.setWildcard(title_buf_);
    add(title_text_);

    /* --------- Content background --------------------------------- */
    content_bg_.setPosition(0, kContentY, kScreenW, kContentH);
    content_bg_.setColor(kColContentBg);
    add(content_bg_);

    /* --------- Menu rows ------------------------------------------ */
    for (int i = 0; i < kMaxRows; ++i)
    {
        int16_t y = kRowsTopY + (int16_t)(i * kRowH);

        row_bg_[i].setPosition(8, y, kScreenW - 16, kRowH - 2);
        row_bg_[i].setColor(kColContentBg);
        row_bg_[i].setVisible(false);
        add(row_bg_[i]);

        row_label_[i].setPosition(16, y + 4, 280, kRowH - 6);
        row_label_[i].setColor(kColText);
        row_label_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", row_label_buf_[i],
                  sizeof(row_label_buf_[i]) / sizeof(row_label_buf_[i][0]));
        row_label_[i].setWildcard(row_label_buf_[i]);
        row_label_[i].setVisible(false);
        add(row_label_[i]);

        row_value_[i].setPosition(300, y + 4, kScreenW - 316, kRowH - 6);
        row_value_[i].setColor(kColTextDim);
        row_value_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", row_value_buf_[i],
                  sizeof(row_value_buf_[i]) / sizeof(row_value_buf_[i][0]));
        row_value_[i].setWildcard(row_value_buf_[i]);
        row_value_[i].setVisible(false);
        add(row_value_[i]);
    }

    /* --------- Value-editor (LARGE centred) ------------------------ */
    editor_value_text_.setPosition(0, kContentY + 40, kScreenW, 60);
    editor_value_text_.setColor(kColRowEdit);
    editor_value_text_.setTypedText(TypedText(T_DYN_L_CENTER));
    utf8ToBuf("", editor_value_buf_,
              sizeof(editor_value_buf_) / sizeof(editor_value_buf_[0]));
    editor_value_text_.setWildcard(editor_value_buf_);
    editor_value_text_.setVisible(false);
    add(editor_value_text_);

    editor_hint_text_.setPosition(0, kScreenH - 28, kScreenW, 20);
    editor_hint_text_.setColor(kColTextDim);
    editor_hint_text_.setTypedText(TypedText(T_DYN_M_CENTER));
    utf8ToBuf("", editor_hint_buf_,
              sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    editor_hint_text_.setWildcard(editor_hint_buf_);
    editor_hint_text_.setVisible(false);
    add(editor_hint_text_);

    /* --------- THERAPY: waveform bars + texts --------------------- */
    {
        const int16_t bars_y    = kContentY + 30;
        const int16_t bars_h    = 100;
        const int16_t bars_left = 20;
        const int16_t bars_w    = kScreenW - 2 * bars_left;
        const int16_t bar_w     = bars_w / kWaveBars;
        for (int i = 0; i < kWaveBars; ++i)
        {
            int16_t x = bars_left + (int16_t)(i * bar_w);
            /* Initial height = bars_h/2, baseline centred. */
            wave_bars_[i].setPosition(x, bars_y + bars_h / 2, bar_w - 2, 4);
            wave_bars_[i].setColor(kColWave);
            wave_bars_[i].setVisible(false);
            add(wave_bars_[i]);
        }

        therapy_pressure_text_.setPosition(0, kContentY + 4, kScreenW, 26);
        therapy_pressure_text_.setColor(kColText);
        therapy_pressure_text_.setTypedText(TypedText(T_DYN_M_CENTER));
        utf8ToBuf("", therapy_pressure_buf_,
                  sizeof(therapy_pressure_buf_) / sizeof(therapy_pressure_buf_[0]));
        therapy_pressure_text_.setWildcard(therapy_pressure_buf_);
        therapy_pressure_text_.setVisible(false);
        add(therapy_pressure_text_);

        therapy_summary_text_.setPosition(0, kScreenH - 30, kScreenW, 22);
        therapy_summary_text_.setColor(kColTextDim);
        therapy_summary_text_.setTypedText(TypedText(T_DYN_M_CENTER));
        utf8ToBuf("", therapy_summary_buf_,
                  sizeof(therapy_summary_buf_) / sizeof(therapy_summary_buf_[0]));
        therapy_summary_text_.setWildcard(therapy_summary_buf_);
        therapy_summary_text_.setVisible(false);
        add(therapy_summary_text_);
    }

    /* --------- WELCOME -------------------------------------------- */
    welcome_big_text_.setPosition(0, kContentY + 30, kScreenW, 60);
    welcome_big_text_.setColor(kColText);
    welcome_big_text_.setTypedText(TypedText(T_DYN_L_CENTER));
    utf8ToBuf("Welcome", welcome_big_buf_,
              sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    welcome_big_text_.setWildcard(welcome_big_buf_);
    welcome_big_text_.setVisible(false);
    add(welcome_big_text_);

    welcome_sub_text_.setPosition(0, kContentY + 110, kScreenW, 26);
    welcome_sub_text_.setColor(kColTextDim);
    welcome_sub_text_.setTypedText(TypedText(T_DYN_M_CENTER));
    utf8ToBuf("Press the knob to begin",
              welcome_sub_buf_,
              sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    welcome_sub_text_.setWildcard(welcome_sub_buf_);
    welcome_sub_text_.setVisible(false);
    add(welcome_sub_text_);
}

void screenView::handleTickEvent()
{
    screenViewBase::handleTickEvent();

    /* Input is owned by Model::tick (runs before this on every frame). We
     * deliberately do NOT poll the rotary here - draining the same latch
     * from two places causes presses to be silently lost when both ticks
     * race for them. */

    /* Therapy waveform animates every tick when running, regardless of any
     * user input. */
    if (cpap::getMenu().therapyRunning() &&
        cpap::getMenu().page() == cpap::Page::THERAPY)
    {
        cpap::getMenu().advanceTherapyPhase();
        updateTherapyWaveform();
    }

    /* Status bar refreshes once per second-ish to show live telemetry. */
    static uint32_t s_status_tick = 0U;
    if (++s_status_tick >= 60U)   /* TouchGFX tick ~ 60 Hz                    */
    {
        s_status_tick = 0U;
        updateStatusBar();
    }

    if (cpap::getMenu().needsRedraw())
    {
        DBG_D("UI", "render page=%u mode=%u sel=%u",
              (unsigned)cpap::getMenu().page(),
              (unsigned)cpap::getMenu().mode(),
              (unsigned)cpap::getMenu().selectedIndex());
        renderCurrentPage();
        cpap::getMenu().clearRedraw();
    }
}

void screenView::updateStatusBar()
{
    using namespace cpap;
    const Settings& s = getSettings();

    /* Left = page name. */
    utf8ToBuf(getMenu().editorTitle(), status_left_buf_,
              sizeof(status_left_buf_) / sizeof(status_left_buf_[0]));
    status_left_text_.invalidate();

    /* Right = airplane / humidifier / connection markers. ASCII-only so we
     * don't need extra glyph ranges in the font. */
    char tmp[40];
    int n = 0;
    if (s.airplane_mode == OnOff::ON)
    {
        n += snprintf(tmp + n, sizeof(tmp) - n, "[AIR] ");
    }
    if (s.humidity != HumidityLevel::OFF)
    {
        n += snprintf(tmp + n, sizeof(tmp) - n, "Hum:%s ", toString(s.humidity));
    }
    if (n == 0)
    {
        snprintf(tmp, sizeof(tmp), "Ready");
    }
    utf8ToBuf(tmp, status_right_buf_,
              sizeof(status_right_buf_) / sizeof(status_right_buf_[0]));
    status_right_text_.invalidate();
}

void screenView::hideAllPageWidgets()
{
    for (int i = 0; i < kMaxRows; ++i)
    {
        row_bg_[i].setVisible(false);
        row_label_[i].setVisible(false);
        row_value_[i].setVisible(false);
    }
    editor_value_text_.setVisible(false);
    editor_hint_text_.setVisible(false);
    welcome_big_text_.setVisible(false);
    welcome_sub_text_.setVisible(false);
    therapy_pressure_text_.setVisible(false);
    therapy_summary_text_.setVisible(false);
    for (int i = 0; i < kWaveBars; ++i)
    {
        wave_bars_[i].setVisible(false);
    }
}

void screenView::renderCurrentPage()
{
    using namespace cpap;
    MenuController& m = getMenu();

    hideAllPageWidgets();
    updateStatusBar();

    /* Title text always reflects the current page. */
    utf8ToBuf(m.editorTitle(), title_buf_,
              sizeof(title_buf_) / sizeof(title_buf_[0]));
    title_text_.invalidate();

    switch (m.page())
    {
    case Page::WELCOME:
        renderWelcomePage();
        break;
    case Page::HOME:
    case Page::MY_OPTIONS:
    case Page::MORE:
        renderListPage();
        break;
    case Page::VALUE_RAMP:
    case Page::VALUE_EPR:
    case Page::VALUE_SMART_START:
    case Page::VALUE_SMART_STOP:
    case Page::VALUE_TUBE_TEMP_MODE:
    case Page::VALUE_TUBE_TEMP_C:
    case Page::VALUE_CLIMATE:
    case Page::VALUE_HUMIDITY:
    case Page::VALUE_AIRPLANE_MODE:
        renderValueEditorPage();
        break;
    case Page::MY_SLEEP_VIEW:
    case Page::MASK_FIT:
    case Page::MYAIR:
        renderInfoPage();
        break;
    case Page::THERAPY:
        renderTherapyPage();
        break;
    default:
        break;
    }

    /* Force the entire content area to repaint so the previous page's pixels
     * are cleared. The Box widgets do the actual clearing; we just invalidate
     * the region. */
    Rect content(0, kContentY, kScreenW, kContentH);
    invalidateRect(content);
}

void screenView::renderListPage()
{
    using namespace cpap;
    MenuController& m = getMenu();
    const uint8_t count = m.itemCount();
    const uint8_t sel   = m.selectedIndex();

    /* Keep the selected item inside the visible window. Note we work in
     * uint8_t throughout - all our list lengths and indices are small. */
    if (sel < list_scroll_offset_)
    {
        list_scroll_offset_ = sel;
    }
    else if (sel >= (uint8_t)(list_scroll_offset_ + kMaxRows))
    {
        list_scroll_offset_ = (uint8_t)(sel - kMaxRows + 1);
    }
    /* Don't leave empty slots at the bottom when the cursor walks back up
     * after a previous scroll. */
    if ((uint16_t)list_scroll_offset_ + kMaxRows > count)
    {
        list_scroll_offset_ = (count > kMaxRows) ? (uint8_t)(count - kMaxRows) : 0U;
    }

    uint8_t visible = (uint8_t)(count - list_scroll_offset_);
    if (visible > kMaxRows) visible = kMaxRows;
    row_visible_count_ = visible;

    /* Hide any rows beyond the visible window left over from a longer list. */
    for (uint8_t r = visible; r < kMaxRows; ++r)
    {
        row_bg_   [r].setVisible(false);
        row_label_[r].setVisible(false);
        row_value_[r].setVisible(false);
    }

    for (uint8_t r = 0; r < visible; ++r)
    {
        const uint8_t i = (uint8_t)(list_scroll_offset_ + r);
        const bool isSel = (i == sel);
        row_bg_[r].setColor(isSel ? kColRowSel : kColContentBg);
        row_bg_[r].setVisible(true);

        utf8ToBuf(m.itemLabel(i), row_label_buf_[r],
                  sizeof(row_label_buf_[r]) / sizeof(row_label_buf_[r][0]));
        row_label_[r].setColor(isSel ? kColBg : kColText);
        row_label_[r].setVisible(true);

        const char* val = m.itemValue(i);
        if (val != nullptr)
        {
            utf8ToBuf(val, row_value_buf_[r],
                      sizeof(row_value_buf_[r]) / sizeof(row_value_buf_[r][0]));
            row_value_[r].setColor(isSel ? kColBg : kColTextDim);
            row_value_[r].setVisible(true);
        }
        else
        {
            row_value_[r].setVisible(false);
        }
    }
}

void screenView::renderValueEditorPage()
{
    using namespace cpap;
    MenuController& m = getMenu();

    utf8ToBuf(m.editorValueText(), editor_value_buf_,
              sizeof(editor_value_buf_) / sizeof(editor_value_buf_[0]));
    editor_value_text_.setVisible(true);

    utf8ToBuf(m.editorHint(), editor_hint_buf_,
              sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    editor_hint_text_.setVisible(true);
}

void screenView::renderInfoPage()
{
    using namespace cpap;
    MenuController& m = getMenu();

    /* Repurpose the welcome widgets for the info text. */
    welcome_big_text_.setColor(kColText);
    welcome_sub_text_.setColor(kColTextDim);

    const char* big = "";
    switch (m.page())
    {
    case Page::MY_SLEEP_VIEW: big = "Sleep view (placeholder)"; break;
    case Page::MASK_FIT:      big = "Mask Fit check";           break;
    case Page::MYAIR:         big = "myAir / Bluetooth";        break;
    default:                  big = "";                         break;
    }
    utf8ToBuf(big, welcome_big_buf_,
              sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    welcome_big_text_.setVisible(true);

    utf8ToBuf(m.editorHint(), welcome_sub_buf_,
              sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    welcome_sub_text_.setVisible(true);
}

void screenView::renderWelcomePage()
{
    utf8ToBuf("Welcome", welcome_big_buf_,
              sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    welcome_big_text_.setColor(kColText);
    welcome_big_text_.setVisible(true);

    utf8ToBuf("Press the knob to begin", welcome_sub_buf_,
              sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    welcome_sub_text_.setColor(kColTextDim);
    welcome_sub_text_.setVisible(true);
}

void screenView::renderTherapyPage()
{
    /* Static texts; bars are animated each tick by updateTherapyWaveform. */
    utf8ToBuf("Pressure: 8.5 cmH2O", therapy_pressure_buf_,
              sizeof(therapy_pressure_buf_) / sizeof(therapy_pressure_buf_[0]));
    therapy_pressure_text_.setVisible(true);

    utf8ToBuf("Hours used: 0:00  Mask: Sealed", therapy_summary_buf_,
              sizeof(therapy_summary_buf_) / sizeof(therapy_summary_buf_[0]));
    therapy_summary_text_.setVisible(true);

    for (int i = 0; i < kWaveBars; ++i)
    {
        wave_bars_[i].setVisible(true);
    }
    updateTherapyWaveform();
}

void screenView::updateTherapyWaveform()
{
    /* Cheap "breath" wave from an integer sine LUT to avoid pulling in the
     * float math library. Scale ranges from -127..+127. */
    static const int8_t kSinLut[60] = {
          0,  13,  26,  39,  52,  64,  76,  87,  97, 106,
        113, 120, 124, 127, 127, 127, 124, 120, 113, 106,
         97,  87,  76,  64,  52,  39,  26,  13,   0, -13,
        -26, -39, -52, -64, -76, -87, -97,-106,-113,-120,
       -124,-127,-127,-127,-124,-120,-113,-106, -97, -87,
        -76, -64, -52, -39, -26, -13,   0,  13,  26,  39
    };
    const int16_t baseline = kContentY + 30 + 100 / 2;
    const int16_t half_h   = 100 / 2 - 8;
    uint32_t phase = cpap::getMenu().therapyTickPhase();

    for (int i = 0; i < kWaveBars; ++i)
    {
        uint32_t idx = (uint32_t)((i + (int)(phase % 60U) * 3) % 60);
        int     v   = kSinLut[idx];
        int     h   = (v * half_h) / 127;
        int16_t y, hh;
        if (h >= 0) { y = (int16_t)(baseline - h); hh = (int16_t)h; }
        else        { y = baseline;                hh = (int16_t)(-h); }
        if (hh < 4) hh = 4;
        Rect old = wave_bars_[i].getRect();
        wave_bars_[i].moveTo(old.x, y);
        wave_bars_[i].setHeight(hh);
        wave_bars_[i].invalidate();
    }
}
