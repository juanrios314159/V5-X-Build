#include "ui.h"
#include "globals.h"

#include "robot/auton.h"
#include "robot/skills.h"
#include "pros/apix.h"
#include "pros/misc.hpp"
#include "pros/imu.hpp"

#include <cstdio>

lv_obj_t* scr_home   = nullptr;
lv_obj_t* scr_auton  = nullptr;
lv_obj_t* scr_skills = nullptr;
lv_obj_t* scr_driver = nullptr;
lv_obj_t* scr_apps   = nullptr;

static lv_obj_t* status_bar        = nullptr;
static lv_obj_t* status_battery_lbl= nullptr;
static lv_obj_t* status_auton_lbl  = nullptr;
static lv_obj_t* status_home_btn   = nullptr;

static lv_style_t style_btn;
static lv_style_t style_btn_active;
static lv_style_t style_screen;

void style_init() {
    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, 8);
    lv_style_set_bg_color(&style_btn, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);
    lv_style_set_border_width(&style_btn, 2);
    lv_style_set_border_color(&style_btn, lv_palette_darken(LV_PALETTE_GREY, 2));
    lv_style_set_text_color(&style_btn, lv_color_white());
    lv_style_set_pad_all(&style_btn, 10);

    lv_style_init(&style_btn_active);
    lv_style_set_bg_color(&style_btn_active, lv_palette_main(LV_PALETTE_GREEN));
    lv_style_set_border_color(&style_btn_active, lv_palette_darken(LV_PALETTE_GREEN, 3));

    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_hex(0x1a1a1a));
    lv_style_set_bg_opa(&style_screen, LV_OPA_COVER);
}

void set_btn_active(lv_obj_t* btn, bool active) {
    if (active) {
        lv_obj_add_style(btn, &style_btn_active, LV_STATE_DEFAULT);
    } else {
        lv_obj_remove_style(btn, &style_btn_active, LV_STATE_DEFAULT);
    }
}

lv_obj_t* make_menu_button(lv_obj_t* parent, const char* text,
                            lv_event_cb_t cb, void* user_data) {
    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_add_style(btn, &style_btn, LV_STATE_DEFAULT);
    lv_obj_set_size(btn, 200, 70);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_center(lbl);

    return btn;
}

void navigate_to(lv_obj_t* target_screen) {
    lv_screen_load(target_screen);

    if (status_home_btn) {
        if (target_screen == scr_home) {
            lv_obj_add_flag(status_home_btn, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_remove_flag(status_home_btn, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void go_home_cb(lv_event_t* e) {
    navigate_to(scr_home);
}

void create_status_bar() {
    status_bar = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(status_bar);
    lv_obj_set_size(status_bar, 480, 30);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(status_bar, LV_OPA_COVER, 0);
    lv_obj_set_flex_flow(status_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(status_bar, LV_FLEX_ALIGN_SPACE_BETWEEN,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_left(status_bar, 8, 0);
    lv_obj_set_style_pad_right(status_bar, 8, 0);
    lv_obj_remove_flag(status_bar, LV_OBJ_FLAG_SCROLLABLE);

    status_home_btn = lv_button_create(status_bar);
    lv_obj_set_size(status_home_btn, 60, 22);
    lv_obj_add_event_cb(status_home_btn, go_home_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_flag(status_home_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t* home_lbl = lv_label_create(status_home_btn);
    lv_label_set_text(home_lbl, "Home");
    lv_obj_center(home_lbl);

    status_auton_lbl = lv_label_create(status_bar);
    lv_obj_set_style_text_color(status_auton_lbl, lv_color_white(), 0);
    lv_label_set_text(status_auton_lbl, "Auton: Do Nothing");

    status_battery_lbl = lv_label_create(status_bar);
    lv_obj_set_style_text_color(status_battery_lbl, lv_color_white(), 0);
    lv_label_set_text(status_battery_lbl, "Batt: --%");
}

void update_status_bar() {
    battery_percent = pros::battery::get_capacity();

    char buf[48];
    std::snprintf(buf, sizeof(buf), "Batt: %.0f%%", battery_percent);
    lv_label_set_text(status_battery_lbl, buf);

    std::snprintf(buf, sizeof(buf), "Auton: %s", selected_auton_name.c_str());
    lv_label_set_text(status_auton_lbl, buf);
}

static void open_auton_cb(lv_event_t* e)  { navigate_to(scr_auton); }
static void open_skills_cb(lv_event_t* e) { navigate_to(scr_skills); }
static void open_driver_cb(lv_event_t* e) { navigate_to(scr_driver); }
static void open_apps_cb(lv_event_t* e)   { navigate_to(scr_apps); }

void create_home_screen() {
    scr_home = lv_obj_create(nullptr);
    lv_obj_add_style(scr_home, &style_screen, LV_STATE_DEFAULT);

    lv_obj_t* grid = lv_obj_create(scr_home);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, 460, 220);
    lv_obj_align(grid, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_EVENLY,
                           LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(grid, LV_OBJ_FLAG_SCROLLABLE);

    make_menu_button(grid, "Autonomous\nSelection", open_auton_cb, nullptr); 
    make_menu_button(grid, "Autonomous\nSkills",    open_skills_cb, nullptr); 
    make_menu_button(grid, "Driver\nPreferences",   open_driver_cb, nullptr); 
    make_menu_button(grid, "Utilities /\nDiagnostics", open_apps_cb, nullptr);
}

static std::vector<lv_obj_t*> auton_buttons;

static void auton_btn_cb(lv_event_t* e) {
    intptr_t idx = reinterpret_cast<intptr_t>(lv_event_get_user_data(e));
    const AutonOption& opt = auton_options[idx];

    selected_auton      = opt.func;
    selected_auton_name = opt.name;

    for (size_t i = 0; i < auton_buttons.size(); i++) {
        set_btn_active(auton_buttons[i], static_cast<intptr_t>(i) == idx);
    }
    update_status_bar();
}

void create_auton_screen() {
    scr_auton = lv_obj_create(nullptr);
    lv_obj_add_style(scr_auton, &style_screen, LV_STATE_DEFAULT);

    lv_obj_t* grid = lv_obj_create(scr_auton);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, 460, 220);
    lv_obj_align(grid, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_EVENLY,
                           LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(grid, LV_OBJ_FLAG_SCROLLABLE);

    auton_buttons.clear();
    for (size_t i = 0; i < auton_options.size(); i++) {
        lv_obj_t* btn = make_menu_button(grid, auton_options[i].name, auton_btn_cb,
                                          reinterpret_cast<void*>(static_cast<intptr_t>(i)));
        lv_obj_set_size(btn, 140, 60);
        auton_buttons.push_back(btn);
        set_btn_active(btn, selected_auton_name == auton_options[i].name);
    }
}

static void skills_btn_cb(lv_event_t* e) {
    intptr_t idx = reinterpret_cast<intptr_t>(lv_event_get_user_data(e));
    const AutonOption& opt = skills_options[idx];
    selected_auton      = opt.func;
    selected_auton_name = opt.name;
    update_status_bar();
}

void create_skills_screen() {
    scr_skills = lv_obj_create(nullptr);
    lv_obj_add_style(scr_skills, &style_screen, LV_STATE_DEFAULT);

    lv_obj_t* col = lv_obj_create(scr_skills);
    lv_obj_remove_style_all(col);
    lv_obj_set_size(col, 300, 200);
    lv_obj_center(col);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(col, 16, 0);
    lv_obj_remove_flag(col, LV_OBJ_FLAG_SCROLLABLE);

    for (size_t i = 0; i < skills_options.size(); i++) {
        make_menu_button(col, skills_options[i].name, skills_btn_cb,
                          reinterpret_cast<void*>(static_cast<intptr_t>(i)));
    }
}

static lv_obj_t* btn_arcade;
static lv_obj_t* btn_tank;
static lv_obj_t* btn_coast;
static lv_obj_t* btn_brake;
static lv_obj_t* btn_hold;

static void drive_mode_cb(lv_event_t* e) {
    lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
    bool is_arcade = (target == btn_arcade);
    driver_settings.drive_mode = is_arcade ? DriveMode::ARCADE : DriveMode::TANK;
    set_btn_active(btn_arcade, is_arcade);
    set_btn_active(btn_tank, !is_arcade);
}

static void brake_mode_cb(lv_event_t* e) {
    lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
    BrakeMode mode = (target == btn_coast) ? BrakeMode::COAST
                    : (target == btn_brake) ? BrakeMode::BRAKE
                                             : BrakeMode::HOLD;
    driver_settings.brake_mode = mode;
    set_btn_active(btn_coast, mode == BrakeMode::COAST);
    set_btn_active(btn_brake, mode == BrakeMode::BRAKE);
    set_btn_active(btn_hold,  mode == BrakeMode::HOLD);
}

void create_driver_screen() {
    scr_driver = lv_obj_create(nullptr);
    lv_obj_add_style(scr_driver, &style_screen, LV_STATE_DEFAULT);

    lv_obj_t* title1 = lv_label_create(scr_driver);
    lv_label_set_text(title1, "Drive Style");
    lv_obj_set_style_text_color(title1, lv_color_white(), 0);
    lv_obj_align(title1, LV_ALIGN_TOP_MID, 0, 45);

    lv_obj_t* row1 = lv_obj_create(scr_driver);
    lv_obj_remove_style_all(row1);
    lv_obj_set_size(row1, 300, 80);
    lv_obj_align(row1, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(row1, LV_OBJ_FLAG_SCROLLABLE);

    btn_arcade = make_menu_button(row1, "Arcade", drive_mode_cb, nullptr);
    btn_tank   = make_menu_button(row1, "Tank",   drive_mode_cb, nullptr);
    lv_obj_set_size(btn_arcade, 120, 60);
    lv_obj_set_size(btn_tank, 120, 60);
    set_btn_active(btn_arcade, driver_settings.drive_mode == DriveMode::ARCADE);
    set_btn_active(btn_tank,   driver_settings.drive_mode == DriveMode::TANK);

    lv_obj_t* title2 = lv_label_create(scr_driver);
    lv_label_set_text(title2, "Motor Brake Mode");
    lv_obj_set_style_text_color(title2, lv_color_white(), 0);
    lv_obj_align(title2, LV_ALIGN_TOP_MID, 0, 155);

    lv_obj_t* row2 = lv_obj_create(scr_driver);
    lv_obj_remove_style_all(row2);
    lv_obj_set_size(row2, 420, 70);
    lv_obj_align(row2, LV_ALIGN_TOP_MID, 0, 180);
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row2, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(row2, LV_OBJ_FLAG_SCROLLABLE);

    btn_coast = make_menu_button(row2, "Coast", brake_mode_cb, nullptr);
    btn_brake = make_menu_button(row2, "Brake", brake_mode_cb, nullptr);
    btn_hold  = make_menu_button(row2, "Hold",  brake_mode_cb, nullptr);
    lv_obj_set_size(btn_coast, 120, 60);
    lv_obj_set_size(btn_brake, 120, 60);
    lv_obj_set_size(btn_hold, 120, 60);
    set_btn_active(btn_coast, driver_settings.brake_mode == BrakeMode::COAST);
    set_btn_active(btn_brake, driver_settings.brake_mode == BrakeMode::BRAKE);
    set_btn_active(btn_hold,  driver_settings.brake_mode == BrakeMode::HOLD);
}

static lv_obj_t* motor_temp_lbls[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};

static void build_motor_temps_app(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(parent, 6, 0);

    static const char* names[5] = {"L Drive", "R Drive", "Intake", "Lift", "Arm"};
    for (int i = 0; i < 5; i++) {
        lv_obj_t* lbl = lv_label_create(parent);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_label_set_text_fmt(lbl, "%s: -- C", names[i]);
        motor_temp_lbls[i] = lbl;
    }
}

static void refresh_motor_temps() {
    if (!motor_temp_lbls[0]) return; 
    double temps[5] = {
        left_DT.get_temperature(0),
        right_DT.get_temperature(0),
        intakeMotors.get_temperature(0),
        liftMotors.get_temperature(0),
        scoreArm.get_temperature(),
    };
    static const char* names[5] = {"L Drive", "R Drive", "Intake", "Lift", "Arm"};
    for (int i = 0; i < 5; i++) {
        lv_label_set_text_fmt(motor_temp_lbls[i], "%s: %.1f C", names[i], temps[i]);
    }
}
static lv_obj_t* imu_heading_lbl = nullptr;

static void imu_reset_cb(lv_event_t* e) {
    imu.reset();
    imu_heading_deg = 0.0;
}

static void build_imu_app(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(parent, 12, 0);

    imu_heading_lbl = lv_label_create(parent);
    lv_obj_set_style_text_color(imu_heading_lbl, lv_color_white(), 0);
    lv_label_set_text_fmt(imu_heading_lbl, "Heading: %.1f deg", imu_heading_deg);

    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_add_event_cb(btn, imu_reset_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* btn_lbl = lv_label_create(btn);
    lv_label_set_text(btn_lbl, "Reset Heading");
    lv_obj_center(btn_lbl);
}

static void refresh_imu() {
    if (!imu_heading_lbl) return;
    imu_heading_deg = imu.get_heading();
    lv_label_set_text_fmt(imu_heading_lbl, "Heading: %.1f deg", imu_heading_deg);
}

static lv_obj_t* distance_lbls[4] = {nullptr, nullptr, nullptr, nullptr};

static void build_distance_app(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(parent, 6, 0);

    static const char* names[4] = {"N", "E", "S", "W"};
    for (int i = 0; i < 4; i++) {
        lv_obj_t* lbl = lv_label_create(parent);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_label_set_text_fmt(lbl, "%s: -- in", names[i]);
        distance_lbls[i] = lbl;
    }
}

static void refresh_distance() {
    if (!distance_lbls[0]) return;
    double in_[4] = {
        dNorth.get_distance() / 25.4,
        dEast.get_distance()  / 25.4,
        dSouth.get_distance() / 25.4,
        dWest.get_distance()  / 25.4,
    };
    static const char* names[4] = {"N", "E", "S", "W"};
    for (int i = 0; i < 4; i++) {
        lv_label_set_text_fmt(distance_lbls[i], "%s: %.2f in", names[i], in_[i]);
    }
}

std::vector<UtilityApp> utility_apps = {
    {"Motor Temps", build_motor_temps_app},
    {"IMU",         build_imu_app},
    {"Distance",    build_distance_app},
};

static void update_diagnostics() {
    refresh_motor_temps();
    refresh_imu();
    refresh_distance();
}

void create_apps_screen() {
    scr_apps = lv_obj_create(nullptr);
    lv_obj_add_style(scr_apps, &style_screen, LV_STATE_DEFAULT);
    lv_obj_t* tabview = lv_tabview_create(scr_apps);
    lv_tabview_set_tab_bar_position(tabview, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tabview, 40);
    lv_obj_set_size(tabview, 480, 235);
    lv_obj_align(tabview, LV_ALIGN_BOTTOM_MID, 0, 0);

    for (const UtilityApp& app : utility_apps) {
        lv_obj_t* tab = lv_tabview_add_tab(tabview, app.name);
        app.build(tab);
    }
}
void ui_init() {
    style_init();

    create_home_screen();
    create_auton_screen();
    create_skills_screen();
    create_driver_screen();
    create_apps_screen();

    create_status_bar();
    update_status_bar();

    navigate_to(scr_home);
}
static void gui_task_fn(void*) {
    uint32_t last_refresh = pros::millis();
    while (true) {
        lv_timer_handler();

        uint32_t now = pros::millis();
        if (now - last_refresh >= 250) { // 4Hz
            update_status_bar();
            update_diagnostics();
            last_refresh = now;
        }
        pros::delay(5);//Dellay it so it doesn't blow
    }
}

void ui_start_background_task() {
    static pros::Task gui_task(gui_task_fn, nullptr, TASK_PRIORITY_DEFAULT - 1,
                                TASK_STACK_DEPTH_DEFAULT, "GUI Task");
}