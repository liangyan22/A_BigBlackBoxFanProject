/*==============================================================================
 * 文件:  stc8h8k64u_bt_bind.c
 * 功能:  STC8H8K64U (22.1184MHz) 蓝牙主/从设备绑定流程示例
 *
 * 硬件接口(注意: 已按 STC8H8K64U 物理引脚修正, 见下方说明):
 *   - 串口1 (P3.0=RXD1, P3.1=TXD1) : 连接 "主机" 蓝牙模块 (AT 主机模式 + BOND)
 *   - 串口2 (P1.0=RXD2, P1.1=TXD2) : 连接 "从机" 蓝牙模块 (AT 从机模式 + 查MAC)
 *   - 按键 SW2 (P3.2, 按下为低电平): 连续有效按下12次触发一次完整绑定流程
 *
 * 修正说明: 在 STC8H8K64U 上, P1.0/P1.1 是串口2的默认脚位(RxD2/TxD2),
 *           P3.0/P3.1 是串口1的默认脚位(RxD1/TxD1)。因此把"从机"放到串口2,
 *           "主机"放到串口1, 角色逻辑(从机配MAC/主机绑定)不变。
 *
 * 波特率: 115200 (22.1184MHz 下可精确分频)
 *   波特率 = SYSclk / 4 / (65536 - 定时器16位重载值)
 *   115200 = 22118400 / 4 / (65536 - 0xFFD0)          (重载差 48 个计数)
 *   故 TH/TL = 0xFF 0xD0
 *
 * 编译环境: Keil C51 (可整文件编译), 寄存器级编程, 轮询方式, 未使用中断。
 *============================================================================*/

#include <reg51.h>
#include <intrins.h>

/*---------------------- 数据类型定义 ----------------------*/
typedef unsigned char  u8;
typedef unsigned int   u16;

/*---------------------- 常数 / 宏定义 ----------------------*/
#define FOSC            22118400UL      /* 主频 22.1184MHz */
#define BT_BAUD         115200          /* 蓝牙 AT 指令波特率 */

/* 按键连击参数 */
#define CLICK_TIMES     12              /* 需连续有效按下次数 */
#define CLICK_WINDOW_MS 1000            /* 两次点击最大间隔, 超时重置计数 */
#define DEBOUNCE_MS     10              /* 按键消抖时间 */

/* AT 指令(仅示例, 以实际蓝牙模块协议为准) */
#define AT_SLAVE_MODE   "AT+ROLE=0\r\n" /* 将蓝牙设为从机 */
#define AT_MASTER_MODE  "AT+ROLE=1\r\n" /* 将蓝牙设为主机 */
#define AT_QUERY_MAC    "AT+MAC?\r\n"   /* 查询 MAC, 返回 +MAC:XXXXXXXXXXXX\r\nOK\r\n */
#define AT_BOND_PREFIX  "AT+BONDMAC="   /* 主机绑定从机MAC指令前缀 */

/*---------------------- 自定义 SFR (reg51.h 未包含) ----------------------*/
sfr P_SW1   = 0xA2;   /* 外设脚位切换控制1 (复位默认=0, 串口1在P3.0/P3.1) */
sfr AUXR    = 0x8E;   /* 辅助寄存器 */
sfr S2CON   = 0x9A;   /* 串口2控制寄存器 */
sfr S2BUF   = 0x9B;   /* 串口2数据寄存器 */
sfr T2H     = 0xD6;   /* 定时器2 高8位(重载值) */
sfr T2L     = 0xD7;   /* 定时器2 低8位(重载值) */

#define S2REN  0x10   /* S2CON.4 允许接收 */
#define S2TI   0x02   /* S2CON.1 发送完成标志 */
#define S2RI   0x01   /* S2CON.0 接收完成标志 */

sbit SW2 = P3^2;      /* 独立按键, 按下=0 */

/*---------------------- 全局变量 ----------------------*/
u8  SlaveMac[12];     /* 12位从机MAC(字符数组, 不含 ':' ) */

/*==============================================================================
 * 函数: delay_ms
 * 说明: 近似毫秒延时, 用于消抖/连击窗口/AT超时(精度要求不高)。
 *============================================================================*/
void delay_ms(u16 ms)
{
    unsigned long cnt;
    while (ms--) {
        cnt = 20300;                 /* 约1ms (22.1184MHz, 含循环开销) */
        while (cnt--) _nop_();
    }
}

/*==============================================================================
 * 串口1 (主机蓝牙, P3.0/P3.1) 初始化: 115200@22.1184, 定时器1做波特率
 *============================================================================*/
void Uart1_Init(void)
{
    P_SW1 = 0x00;                    /* 保持默认: 串口1 -> P3.0(RXD)/P3.1(TXD) */

    SCON = 0x50;                     /* 8位数据, 方式1, 可变波特率, REN=1 */

    AUXR |= 0x40;                    /* 定时器时钟 1T 模式(串口1波特率发生器) */
    AUXR &= 0xFE;                    /* 串口1选择定时器1做波特率发生器 */
    TMOD &= 0x0F;                    /* 定时器1: 方式0 (16位自动重载) */

    TH1 = 0xFF;  TL1 = 0xD0;         /* 115200 @ 22.1184MHz, 重载值 0xFFD0 */
    TR1 = 1;                         /* 启动定时器1 */
}

/*==============================================================================
 * 串口2 (从机蓝牙, P1.0/P1.1) 初始化: 115200@22.1184, 定时器2做波特率
 *============================================================================*/
void Uart2_Init(void)
{
    S2CON = S2REN;                   /* S2REN=1 允许接收 */

    T2L = 0xD0;  T2H = 0xFF;         /* 115200 @ 22.1184MHz, 重载值 0xFFD0 */

    AUXR |= 0x04;                    /* 定时器2 1T 模式 */
    AUXR |= 0x10;                    /* 启动定时器2 (T2R) */
}

/*==============================================================================
 * 串口1 单字节收发 (主机)
 *============================================================================*/
void Uart1_SendByte(u8 dat)
{
    SBUF = dat;
    while (!TI);                     /* 等待发送完成 */
    TI = 0;
}
u8 Uart1_ReceiveByte(u8 *dat)        /* 返回1=收到一字节 */
{
    if (RI) { *dat = SBUF; RI = 0; return 1; }
    return 0;
}

/*==============================================================================
 * 串口2 单字节收发 (从机)
 *============================================================================*/
void Uart2_SendByte(u8 dat)
{
    S2BUF = dat;
    while (!(S2CON & S2TI));         /* 等待发送完成 */
    S2CON &= ~S2TI;
}
u8 Uart2_ReceiveByte(u8 *dat)        /* 返回1=收到一字节 */
{
    if (S2CON & S2RI) { *dat = S2BUF; S2CON &= ~S2RI; return 1; }
    return 0;
}

void Uart1_SendStr(const u8 *s){ while (*s) Uart1_SendByte(*s++); }
void Uart2_SendStr(const u8 *s){ while (*s) Uart2_SendByte(*s++); }

/*==============================================================================
 * 函数: IsOkReceived
 * 说明: 判断缓冲区中是否出现子串 "OK"。
 *============================================================================*/
static u8 IsOkReceived(const u8 *buf, u16 len)
{
    u16 i;
    for (i = 0; i + 1 < len; i++)
        if (buf[i] == 'O' && buf[i + 1] == 'K') return 1;
    return 0;
}

/*==============================================================================
 * 函数: Uart1_ReadFrame / Uart2_ReadFrame
 * 说明: 阻塞读取一帧回复, 直到出现 "OK" 或超时。
 * 返回: 实际收到字节数; 缓冲以 '\0' 结尾。
 *============================================================================*/
u16 Uart1_ReadFrame(u8 *buf, u16 size, u16 tmoMs)
{
    u16 i = 0, t = 0;
    u8  c;
    while (t < tmoMs) {
        if (Uart1_ReceiveByte(&c)) {
            if (i < size - 1) { buf[i++] = c; buf[i] = 0; }
            if (IsOkReceived(buf, i)) break;
        } else {
            delay_ms(1); t++;
        }
    }
    return i;
}

u16 Uart2_ReadFrame(u8 *buf, u16 size, u16 tmoMs)
{
    u16 i = 0, t = 0;
    u8  c;
    while (t < tmoMs) {
        if (Uart2_ReceiveByte(&c)) {
            if (i < size - 1) { buf[i++] = c; buf[i] = 0; }
            if (IsOkReceived(buf, i)) break;
        } else {
            delay_ms(1); t++;
        }
    }
    return i;
}

/*==============================================================================
 * 函数: IsHex
 *============================================================================*/
static u8 IsHex(u8 ch)
{
    return ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F') || (ch >= 'a' && ch <= 'f'));
}

/*==============================================================================
 * 函数: ExtractMac
 * 说明: 在返回帧中查找子串 "MAC:" 并提取其后 12 位十六进制 MAC。
 * 输入: 帧 "+MAC:XXXXXXXXXXXX\r\nOK\r\n"
 * 返回: 1=成功, 0=失败
 *============================================================================*/
u8 ExtractMac(const u8 *buf, u16 len, u8 *mac)
{
    u16 i;
    u8  cnt = 0;
    for (i = 0; i < len; i++) {
        if (buf[i] == 'M' && buf[i + 1] == 'A' && buf[i + 2] == 'C' && buf[i + 3] == ':') {
            i += 4;                       /* 跳过 "MAC:" 冒号 */
            while (cnt < 12 && i < len) {
                u8 ch = buf[i];
                if (!IsHex(ch)) return 0; /* 遇到非法字符, MAC无效 */
                mac[cnt++] = ch;
                i++;
            }
            return (cnt == 12);
        }
    }
    return 0;
}

/*==============================================================================
 * 函数: BuildBondCmd
 * 说明: 拼装主机绑定指令 "AT+BONDMAC=XXXXXXXXXXXX\r\n"。
 *============================================================================*/
void BuildBondCmd(u8 *out, const u8 *mac)
{
    const u8 *pre = AT_BOND_PREFIX;
    u8 i = 0, j = 0;
    while (pre[i]) out[j++] = pre[i++];
    for (i = 0; i < 12; i++) out[j++] = mac[i];
    out[j++] = '\r'; out[j++] = '\n'; out[j] = 0;
}

/*==============================================================================
 * 函数: Uart1_Flush / Uart2_Flush
 * 说明: 清空接收标志, 丢弃残留数据。
 *============================================================================*/
void Uart1_Flush(void){ RI = 0; }
void Uart2_Flush(void){ S2CON &= ~S2RI; }

/*==============================================================================
 * 函数: Bluetooth_Bind_Flow
 * 说明: 完整蓝牙绑定流程:
 *       1) 串口2(从机): AT置从机模式, AT+MAC? 查询并解析12位从机MAC
 *       2) 串口1(主机): AT置主机模式, AT+BONDMAC=解析到的从机MAC, 收到OK为绑定成功
 *============================================================================*/
void Bluetooth_Bind_Flow(void)
{
    u8 buf[64];
    u16 n;
    u8 bondDone = 0;

    /* ---- 1) 从机蓝牙(串口2) 设为从机模式 ---- */
    Uart2_Flush();
    Uart2_SendStr(AT_SLAVE_MODE);              /* 例如 AT+ROLE=0 */
    n = Uart2_ReadFrame(buf, sizeof(buf), 500);/* 等 OK */
    if (n == 0 || !IsOkReceived(buf, n)) return;   /* 从机AT无响应则中止 */

    /* ---- 2) 从机蓝牙(串口2) 查询MAC并解析 ---- */
    Uart2_Flush();
    Uart2_SendStr(AT_QUERY_MAC);               /* AT+MAC? */
    n = Uart2_ReadFrame(buf, sizeof(buf), 500);
    if (!ExtractMac(buf, n, SlaveMac)) return; /* 解析 12 位从机 MAC 失败则中止 */

    /* ---- 3) 主机蓝牙(串口1) 设为主机模式 ---- */
    Uart1_Flush();
    Uart1_SendStr(AT_MASTER_MODE);             /* 例如 AT+ROLE=1 */
    n = Uart1_ReadFrame(buf, sizeof(buf), 500);
    if (n == 0 || !IsOkReceived(buf, n)) return;

    /* ---- 4) 主机蓝牙(串口1) 绑定从机MAC ---- */
    BuildBondCmd(buf, SlaveMac);               /* AT+BONDMAC=XXXXXXXXXXXX */
    Uart1_Flush();
    Uart1_SendStr(buf);
    n = Uart1_ReadFrame(buf, sizeof(buf), 800);
    bondDone = (n != 0 && IsOkReceived(buf, n));  /* 收到 OK 判定绑定成功 */

    if (bondDone) {
        /* 绑定成功: 这里可加指示灯/回调等业务处理 */
    }
}

/*==============================================================================
 * 函数: ReadPress / DebouncedPress / ClickDetect / TryClickWithin
 * 说明: 按键消抖与连击(连点)逻辑。
 *============================================================================*/
static u8 ReadPress(void){ return !SW2; }

static u8 DebouncedPress(void)      /* 消抖后确认按下 */
{
    if (!ReadPress()) return 0;
    delay_ms(DEBOUNCE_MS);
    return ReadPress();
}

static u8 ClickDetect(void)         /* 一次完整点击: 消抖按下 + 等待释放 */
{
    if (!DebouncedPress()) return 0;
    while (ReadPress()) delay_ms(1);    /* 等待松开 */
    return 1;
}

static u8 TryClickWithin(u16 windowMs) /* 在窗口内等待下一次有效按下 */
{
    u16 t = 0;
    while (t < windowMs) {
        if (DebouncedPress()) {
            while (ReadPress()) delay_ms(1); /* 等待松开 */
            return 1;
        }
        delay_ms(1); t++;
    }
    return 0;
}

/*==============================================================================
 * 主函数
 *============================================================================*/
void main(void)
{
    u8 clicks;

    Uart1_Init();                    /* 主机蓝牙 */
    Uart2_Init();                    /* 从机蓝牙 */

    SW2 = 1;                         /* P3.2 置高, 配合准双向读输入 */

    for (;;) {
        clicks = 0;

        /* 第一次有效点击 */
        if (!ClickDetect()) {
            delay_ms(1);
            continue;
        }
        clicks = 1;

        /* 在连击窗口内继续按键, 累计有效次数 */
        while (TryClickWithin(CLICK_WINDOW_MS)) {
            clicks++;
            if (clicks >= CLICK_TIMES) break;
        }

        /* 若达到连续12次有效按下, 触发完整蓝牙绑定流程 */
        if (clicks >= CLICK_TIMES) {
            Bluetooth_Bind_Flow();
        }

        delay_ms(50);                /* 防抖间隙, 防止流程结束后误触发 */
    }
}