#include "usmart.h"
#include "usmart_str.h"
#include "app_config.h"

#if FEATURE_USMART_ENABLE

/******************************************************************************************/
/* 用户配置区
 * 这下面要包含所用到的函数所申明的头文件(用户自己添加)
 */

#include "sys.h"
#include "delay.h"
#include "usmart_interface.h"  /* USMART 可调用函数接口 */

/* 函数名列表初始化(用户自己添加)
 * 用户直接在这里输入要执行的函数名及其查找串
 * 
 * 注: 函数指针转换为void*会产生-Wpedantic警告，属于USMART组件设计
 * 此警告不影响功能，可在CMakeLists.txt中全局禁用或忽略
 */
struct _m_usmart_nametab usmart_nametab[] =
{
#if USMART_USE_WRFUNS == 1      /* 如果使能了读写操作 */
    {(void *)read_addr, "uint32_t read_addr(uint32_t addr)"},
    {(void *)write_addr, "void write_addr(uint32_t addr,uint32_t val)"},
#endif
    /* SYSTEM 延时函数 */
    {(void *)delay_ms, "void delay_ms(uint16_t nms)"},
    {(void *)delay_us, "void delay_us(uint32_t nus)"},
    
    /* BSP/LED 模块 */
    {(void *)led_init, "void led_init(void)"},
    
    /* BSP/KEY 模块 */
    {(void *)key_init, "void key_init(void)"},
    {(void *)key_scan, "uint8_t key_scan(uint8_t mode)"},
    
    /* BSP/IWDG 模块 */
    {(void *)iwdg_init, "void iwdg_init(uint8_t prer,uint16_t rlr)"},
    {(void *)iwdg_feed, "void iwdg_feed(void)"},
    
    /* BSP/EMM_V5 电机控制模块 */
    {(void *)motor_enable, "void motor_enable(uint8_t addr,uint8_t enable)"},
    {(void *)motor_pos_move, "void motor_pos_move(uint8_t addr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses)"},
    {(void *)motor_vel_move, "void motor_vel_move(uint8_t addr,uint8_t dir,uint16_t speed,uint8_t acc)"},
    {(void *)motor_stop, "void motor_stop(uint8_t addr)"},
    {(void *)motor_home, "void motor_home(uint8_t addr)"},
    {(void *)motor_read_status, "void motor_read_status(uint8_t addr)"},
    
    /* V3.1: 多电机管理模块 */
    {(void *)multi_scan, "void multi_scan(uint8_t start,uint8_t end)"},
    {(void *)multi_map, "void multi_map(uint8_t modbus,uint8_t physical)"},
    {(void *)multi_list, "void multi_list(void)"},
    {(void *)multi_enable, "void multi_enable(uint16_t mask,uint8_t enable)"},
    {(void *)multi_pos, "void multi_pos(uint16_t mask,uint8_t dir,uint16_t speed,uint32_t pulses)"},
    {(void *)multi_vel, "void multi_vel(uint16_t mask,uint8_t dir,uint16_t speed)"},
    {(void *)multi_stop, "void multi_stop(uint16_t mask)"},
    {(void *)multi_home, "void multi_home(uint16_t mask,uint8_t mode)"},
    
    /* V3.1: 协议统计模块 */
    {(void *)proto_stats, "void proto_stats(void)"},
    {(void *)proto_reset, "void proto_reset(void)"},
    
    /* V3.5 Phase 8 P1: 增量CRC调试模块 */
    {(void *)crc_stats, "void crc_stats(void)"},
    {(void *)fifo_stats, "void fifo_stats(void)"},
};

/******************************************************************************************/

/* 函数控制管理器初始化
 * 得到各个受控函数的名字
 * 得到函数总数量
 */
struct _m_usmart_dev usmart_dev =
{
    .funs = usmart_nametab,
    .init = usmart_init,
    .cmd_rec = usmart_cmd_rec,
    .exe = usmart_exe,
    .scan = usmart_scan,
    .fnum = sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), /* 函数数量 */
    .pnum = 0,      /* 参数数量 */
    .id = 0,        /* 函数ID */
    .sptype = 1,    /* 参数显示类型,0,10进制;1,16进制 */
    .parmtype = 0,  /* 参数类型.bitx:,0,数字;1,字符串 */
    .plentbl = {0}, /* 每个参数的长度暂存表,需要MAX_PARM个0初始化 */
    .parm = {0},    /* 函数的参数,需要PARM_LEN个0初始化 */
};

#else  /* !FEATURE_USMART_ENABLE */

/* USMART禁用时提供占位实现 */
struct _m_usmart_nametab usmart_nametab[] = { {0, "USMART_DISABLED"} };

struct _m_usmart_dev usmart_dev =
{
    .funs = usmart_nametab,
    .init = 0,
    .cmd_rec = 0,
    .exe = 0,
    .scan = 0,
    .fnum = 0,
    .pnum = 0,
    .id = 0,
    .sptype = 1,
    .parmtype = 0,
    .plentbl = {0},
    .parm = {0},
};

#endif  /* FEATURE_USMART_ENABLE */



















