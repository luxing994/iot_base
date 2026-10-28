#ifndef FILE_TRANSFER_H
#define FILE_TRANSFER_H

#include <stdint.h>

/*
 * 上位机文件分包传输（文件端口 CONFIG_IOT_FILE_PORT）
 *
 * 先发一个控制包 "<文件名><分隔符><文件大小>"（分隔符为 "::!!@@$$**" 中的任意字符），
 * 之后按 FILETRANSSIZE 字节一包发送文件内容。每收到一包数据通知一次使用方，
 * 使用方通过 GetFileData() 取走当前包。
 */
void FileTransfer_Reset(void);      // 新连接开始时调用：丢弃上一次未传完的文件
void FileTransfer_HandlePacket(char *data, int len);

int GetFileCount(uint8_t *count);
int GetFileData(uint8_t *data, int size);

/* 乐江罗拉车配置项（本分支无写入方，保留给相关设备分支使用） */
int GetTaskNum(uint16_t *tasknum);
int GetTaskPitch(uint16_t *taskpitch);
int GetTaskSpeed(uint16_t *taskspeed);
int GetTaskCount(uint16_t *taskcount);
int GetTaskTime(uint16_t *tasktime);
int GetMode(uint8_t *mode);

#endif
