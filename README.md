# STM32 SBUS Driver for WFLY ET16S

[中文](#中文) | [English](#english)

---

# 中文

## 项目简介

这是一个适用于 **STM32** 的天地飞（WFLY）ET16S **SBUS 接收驱动**。

项目实现了完整的 SBUS 接收流程，包括：

- USART3 + DMA 双缓冲接收
- UART 空闲中断触发帧解析
- SBUS 25 字节数据解析
- 16 路 11 bit 通道位运算解包
- 两个数字通道及 FailSafe 状态解析
- 摇杆中点归零
- 拨杆状态转换
- 拨轮值映射
- 输出统一的 `RC_ctrl_t` 与 `data_rc_t` 数据结构

该驱动设计为独立模块，可直接移植到 STM32 工程。

## 功能特点

- 双缓冲 DMA，提高连续接收可靠性
- 基于 UART IDLE 中断判断一帧结束
- 使用位运算解析 SBUS 数据
- 将原始通道转换为更易使用的数据结构
- 提供初始化、数据读取和控制量接口

## 文件

```text
wfly_et16s.h
wfly_et16s.c
```

## API

```c
void RC_Init(void);
const RC_ctrl_t *get_remote_control_point(void);
void rc_read_data(data_rc_t *data_rc);
```

## 工作流程

```
SBUS
   │
UART3 DMA
   │
Double Buffer
   │
IDLE Interrupt
   │
SBUS Frame Parser
   │
RC_ctrl_t
   │
Application
```

## 数据内容

- 16 路模拟通道
- SA~SH 开关
- 左右拨轮
- CH17 / CH18
- Frame Lost
- FailSafe

## 平台

- MCU：STM32
- HAL Library
- USART3
- DMA Double Buffer

---

# English

## Overview

A lightweight SBUS receiver driver for the WFLY ET16S transmitter on STM32.

Features include:

- USART + DMA double buffer reception
- UART IDLE interrupt
- Complete 25-byte SBUS frame parser
- 16-channel (11-bit) extraction
- Switch and wheel decoding
- Stick center normalization
- Output through clean C structures

## Features

- DMA Double Buffer
- Interrupt Driven Reception
- Bitwise SBUS Decoder
- RC Structure Output
- Easy Integration

## File Structure

```text
wfly_et16s.h
wfly_et16s.c
```

## Public APIs

```c
void RC_Init(void);
const RC_ctrl_t *get_remote_control_point(void);
void rc_read_data(data_rc_t *data_rc);
```


## Notes

This project demonstrates embedded software techniques including DMA, interrupt-driven communication, bit-field parsing and protocol abstraction.
