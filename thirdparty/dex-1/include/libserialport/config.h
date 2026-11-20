/**
 * @file config.h
 * @brief libserialport 库配置文件
 *
 * 此文件属于主项目，而非 libserialport 子模块。
 * 提供 libserialport 支持非标准波特率（4M/6M）所需的配置，
 * 用于宇树电机通信。
 */

#ifndef LIBSERIALPORT_CONFIG_H /**< 防止重复包含 */
#define LIBSERIALPORT_CONFIG_H

/**
 * @name 导出宏定义
 * 库编译时的符号可见性控制
 * @{
 */
#define SP_API                          /**< 公共API导出宏 */
#define SP_PRIV __attribute__((visibility("hidden"))) /**< 私有符号隐藏宏 */
/** @} */

/**
 * @name 平台能力检测
 * 系统头文件可用性定义
 * @{
 */
#define HAVE_FCNTL_H 1          /**< 文件控制选项头文件 */
#define HAVE_UNISTD_H 1         /**< UNIX标准函数头文件 */
#define HAVE_SYS_IOCTL_H 1      /**< I/O控制头文件 */
#define HAVE_SYS_TIME_H 1       /**< 时间类型头文件 */
#define HAVE_TERMIOS_H 1        /**< 终端I/O头文件 */
#define HAVE_LINUX_TERMIOS_H 1  /**< Linux终端I/O头文件 */
#define HAVE_SYS_EVENTFD_H 1    /**< 事件文件描述符头文件 */
#define HAVE_EVENTFD 1          /**< 支持eventfd系统调用 */
#define HAVE_CLOCK_GETTIME 1    /**< 支持高精度时钟获取 */
#define HAVE_REALPATH 1         /**< 支持路径解析 */
/** @} */

/**
 * @name Linux termios2支持
 * 用于设置非标准波特率（如4Mbps、6Mbps）
 * @{
 */
#define HAVE_TCGETS2 1          /**< 支持TCGETS2 ioctl获取串口配置 */
#define HAVE_DECL_BOTHER 1      /**< 支持BOTHER标志自定义波特率 */
/** @} */

/**
 * @name Termios2波特率支持
 * 启用termios2结构体以支持非标准波特率（宇树电机需要4M/6M波特率）
 * @{
 */
#define HAVE_STRUCT_TERMIOS2 1              /**< 支持termios2结构体 */
#define HAVE_STRUCT_TERMIOS2_C_ISPEED 1     /**< 支持输入波特率成员 */
#define HAVE_STRUCT_TERMIOS2_C_OSPEED 1     /**< 支持输出波特率成员 */
#define HAVE_TERMIOS2_SPEED 1               /**< 支持通过c_ispeed/c_ospeed设置任意波特率 */
/** @} */

/**
 * @name 类型定义
 * 基本数据类型支持
 * @{
 */
#define HAVE_SSIZE_T 1          /**< 支持ssize_t有符号大小类型 */
/** @} */

#endif /* LIBSERIALPORT_CONFIG_H */ /**< 结束防止重复包含 */
