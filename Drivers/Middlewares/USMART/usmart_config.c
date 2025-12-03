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
#include "y_v2.h"               /* V3.0: Y系列X固件协议驱动 */

/* 函数名列表初始化(用户自己添加)
 * 用户直接在这里输入要执行的函数名及其查找串
 * 
 * 注: 函数指针转换为void*会产生-Wpedantic警告，属于USMART组件设计
 * 此警告不影响功能，可在CMakeLists.txt中全局禁用或忽略
 */
struct _m_usmart_nametab usmart_nametab[] =
#ifdef printer_clear_emergency_stop
    {(void *)printer_clear_emergency_stop, "void printer_clear_emergency_stop(void)"},
#endif
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
    /* {(void *)key_init, "void key_init(void)"},  // 如未用可注释 */
    
    /* BSP/IWDG 模块 */
    {(void *)iwdg_init, "void iwdg_init(uint8_t prer,uint16_t rlr)"},
    {(void *)iwdg_feed, "void iwdg_feed(void)"},
    /* BSP/Y_V2 Y系列X固件电机控制（V3.0协议升级）*/
    {(void *)Y_V2_En_Control, "void Y_V2_En_Control(uint8_t addr,uint8_t state,uint8_t snF)"},
    {(void *)Y_V2_Bypass_Pos_Control, "void Y_V2_Bypass_Pos_Control(uint8_t addr,uint8_t dir,float vel,float pos,uint8_t raf,uint8_t snF)"},
    {(void *)Y_V2_Vel_Control, "void Y_V2_Vel_Control(uint8_t addr,uint8_t dir,uint16_t acc,float vel,uint8_t snF)"},
    {(void *)Y_V2_Stop_Now, "void Y_V2_Stop_Now(uint8_t addr,uint8_t snF)"},
    {(void *)Y_V2_Origin_Trigger_Return, "void Y_V2_Origin_Trigger_Return(uint8_t addr,uint8_t mode,uint8_t snF)"},
    {(void *)Y_V2_Read_Sys_Params, "void Y_V2_Read_Sys_Params(uint8_t addr,SysParams_t param)"},
    {(void *)Y_V2_Synchronous_Motion, "void Y_V2_Synchronous_Motion(uint8_t addr)"},
    /* 3D打印机3轴控制（高层）*/
    {(void *)printer_enable_all, "void printer_enable_all(void)"},
    {(void *)printer_disable_all, "void printer_disable_all(void)"},
    /* 脉冲数单位移动（底层调试）*/
    {(void *)printer_move_x, "void printer_move_x(int32_t distance,uint16_t speed)"},
    {(void *)printer_move_y, "void printer_move_y(int32_t distance,uint16_t speed)"},
    {(void *)printer_move_z, "void printer_move_z(int32_t distance,uint16_t speed)"},
    {(void *)printer_move_xyz, "void printer_move_xyz(int32_t x,int32_t y,int32_t z,uint16_t speed)"},
    /* 毫米单位移动（应用层推荐，USMART兼容整数版本）*/
    {(void *)printer_move_x_mm_int, "void printer_move_x_mm_int(int16_t distance_dmm,uint16_t speed)"},
    {(void *)printer_move_y_mm_int, "void printer_move_y_mm_int(int16_t distance_dmm,uint16_t speed)"},
    {(void *)printer_move_z_mm_int, "void printer_move_z_mm_int(int16_t distance_dmm,uint16_t speed)"},
    {(void *)printer_xyz_mm_int, "void printer_xyz_mm_int(int16_t x_dmm,int16_t y_dmm,int16_t z_dmm,uint16_t speed)"},
    /* 回零与状态 */
    {(void *)printer_home_x, "void printer_home_x(void)"},
    {(void *)printer_home_y, "void printer_home_y(void)"},
    {(void *)printer_home_z, "void printer_home_z(void)"},
    {(void *)printer_home_all_axes, "void printer_home_all_axes(void)"},
    {(void *)printer_estop, "void printer_estop(void)"},
    {(void *)printer_show_status, "void printer_show_status(void)"},
    /* 调试统计模块 */
    {(void *)crc_stats, "void crc_stats(void)"},
    {(void *)fifo_stats, "void fifo_stats(void)"},
    {(void *)motor_monitor_status, "void motor_monitor_status(void)"},  /* V3.7: 电机监控 */
    /* V3.7: 帮助系统 */
    {(void *)motor_help, "void motor_help(void)"},  /* 电机命令帮助 */
    
    /* 毫米单位移动（应用层推荐，USMART兼容整数版本）*/
    {(void *)printer_move_x_mm_int, "void printer_move_x_mm_int(int16_t distance_dmm,uint16_t speed)"},
    {(void *)printer_move_y_mm_int, "void printer_move_y_mm_int(int16_t distance_dmm,uint16_t speed)"},
    {(void *)printer_move_z_mm_int, "void printer_move_z_mm_int(int16_t distance_dmm,uint16_t speed)"},
    {(void *)printer_xyz_mm_int, "void printer_xyz_mm_int(int16_t x_dmm,int16_t y_dmm,int16_t z_dmm,uint16_t speed)"},
    
    /* 回零与状态 */
    {(void *)printer_home_x, "void printer_home_x(void)"},
    {(void *)printer_home_y, "void printer_home_y(void)"},
    {(void *)printer_home_z, "void printer_home_z(void)"},
    {(void *)printer_home_all_axes, "void printer_home_all_axes(void)"},
    {(void *)printer_estop, "void printer_estop(void)"},
    {(void *)printer_show_status, "void printer_show_status(void)"},
    
    /* 调试统计模块 */
    {(void *)crc_stats, "void crc_stats(void)"},
    {(void *)fifo_stats, "void fifo_stats(void)"},
    {(void *)motor_monitor_status, "void motor_monitor_status(void)"},  /* V3.7: 电机监控 */
    
    /* V3.7: 帮助系统 */
    {(void *)motor_help, "void motor_help(void)"},  /* 电机命令帮助 */
    
#if REALTIME_MOTOR_ENABLE
    /* V3.6: TIM2实时定时器控制 */
    {(void *)tim2_start, "void tim2_start(void)"},
    {(void *)tim2_stop, "void tim2_stop(void)"},
    {(void *)tim2_status, "void tim2_status(void)"},
#endif
    
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



















