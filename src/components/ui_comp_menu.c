// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "../ui.h"


// COMPONENT Menu

lv_obj_t * ui_Menu_create(lv_obj_t * comp_parent)
{

    lv_obj_t * cui_Menu;
    cui_Menu = lv_btn_create(comp_parent);
    lv_obj_set_width(cui_Menu, 41);
    lv_obj_set_height(cui_Menu, 31);
    lv_obj_set_x(cui_Menu, 132);
    lv_obj_set_y(cui_Menu, 98);
    lv_obj_set_align(cui_Menu, LV_ALIGN_CENTER);
    lv_obj_add_flag(cui_Menu, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(cui_Menu, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    lv_obj_t ** children = lv_mem_alloc(sizeof(lv_obj_t *) * _UI_COMP_MENU_NUM);
    children[UI_COMP_MENU_MENU] = cui_Menu;
    lv_obj_add_event_cb(cui_Menu, get_component_child_event_cb, LV_EVENT_GET_COMP_CHILD, children);
    lv_obj_add_event_cb(cui_Menu, del_component_child_event_cb, LV_EVENT_DELETE, children);
    ui_comp_Menu_create_hook(cui_Menu);
    return cui_Menu;
}

