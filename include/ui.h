#pragma once
#include "liblvgl/lvgl.h"
#include <vector>
#include <string>
#include <functional>
#include "globals.h"

void ui_init();
void ui_start_background_task();

extern lv_obj_t* scr_home;
extern lv_obj_t* scr_auton;
extern lv_obj_t* scr_skills;
extern lv_obj_t* scr_driver;
extern lv_obj_t* scr_apps;

void create_home_screen();
void create_auton_screen();
void create_skills_screen();
void create_driver_screen();
void create_apps_screen();


void create_status_bar();
void update_status_bar();  

void navigate_to(lv_obj_t* target_screen);

struct UtilityApp {
    const char* name;
    std::function<void(lv_obj_t* parent)> build;
};
extern std::vector<UtilityApp> utility_apps;

void style_init();
void set_btn_active(lv_obj_t* btn, bool active);

lv_obj_t* make_menu_button(lv_obj_t* parent, const char* text,
                            lv_event_cb_t cb, void* user_data);