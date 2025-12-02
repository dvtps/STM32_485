#include "multi_motor_manager.h"
#include <string.h>

static motor_manager_t g_mgr = {0};

HAL_StatusTypeDef motor_mgr_init(void) {
    memset(&g_mgr, 0, sizeof(g_mgr));
    g_mgr.initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef motor_mgr_add(uint8_t addr) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_remove(uint8_t addr) { return HAL_OK; }
motor_state_t* motor_mgr_find(uint8_t addr) { return NULL; }
uint8_t motor_mgr_get_count(void) { return g_mgr.motor_count; }
uint8_t motor_mgr_scan(uint8_t s, uint8_t e) { return 0; }
HAL_StatusTypeDef motor_mgr_query_enable(uint8_t a) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_query_position(uint8_t a) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_query_velocity(uint8_t a) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_query_vbus(uint8_t a) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_update_all_status(void) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_enable(uint8_t a, bool e) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_move(uint8_t a, uint8_t d, uint16_t s, uint8_t ac, uint32_t p, bool r) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_stop(uint8_t a) { return HAL_OK; }
HAL_StatusTypeDef motor_mgr_home(uint8_t a, uint8_t m) { return HAL_OK; }
uint8_t motor_mgr_enable_all(void) { return 0; }
uint8_t motor_mgr_enable_batch(uint8_t *a, uint8_t c) { return 0; }
uint8_t motor_mgr_stop_all(void) { return 0; }
HAL_StatusTypeDef motor_mgr_sync_move(motor_move_t *m, uint8_t c) { return HAL_OK; }
bool motor_mgr_is_timeout(uint8_t a) { return false; }
uint8_t motor_mgr_get_health(uint8_t a) { return 100; }
void motor_mgr_check_timeout(void) {}
void motor_mgr_auto_recover(void) {}
HAL_StatusTypeDef motor_mgr_clear_errors(uint8_t a) { return HAL_OK; }
void motor_mgr_print_status(uint8_t a) {}
void motor_mgr_print_all_status(void) {}
void motor_mgr_get_statistics(uint32_t *c, uint32_t *e) { *c=0; *e=0; }

/* V3.1兼容接口 */
int multi_motor_scan(uint8_t s, uint8_t e) { return motor_mgr_scan(s,e); }
int multi_motor_map_address(uint8_t m, uint8_t p) { return 0; }
void multi_motor_print_list(void) { motor_mgr_print_all_status(); }
void multi_motor_enable_batch(uint16_t mask, bool en) {}
void multi_motor_pos_control_batch(uint16_t mask, uint8_t d, uint16_t s, uint8_t a, uint32_t p) {}
void multi_motor_vel_control_batch(uint16_t mask, uint8_t d, uint16_t s, uint8_t a) {}
void multi_motor_stop_batch(uint16_t mask) {}
void multi_motor_home_batch(uint16_t mask, uint8_t m) {}
void multi_motor_poll(void) { motor_mgr_update_all_status(); }
