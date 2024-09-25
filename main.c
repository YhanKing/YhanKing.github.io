/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "public.h"
#include "algorithm.h"

#define MAX_PATH_LENGTH 256

/* 读取文件内容并解析 */
int parseFile(const char *filename, HeadInfo *headInfo, IOVector *ioVector)
{
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open file");
        return RETURN_ERROR;
    }

    char line[256];
    int ioCount = 0;
    IOUint *ioArray = NULL;

    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "[\"head", 6) == 0) {
            // 读取 head 信息
            if (fgets(line, sizeof(line), file)) {
                sscanf(line, "[%u,%u,%u]", &headInfo->wrap, &headInfo->lpos, (uint32_t *)&headInfo->status);
                printf("head info : %s\n", line);
            }
        } else if (strncmp(line, "[\"io count", 10) == 0) {
            // 读取 io count 信息
            if (fgets(line, sizeof(line), file)) {
                sscanf(line, "[%u]", &ioVector->len);
                ioArray = (IOUint *)malloc(ioVector->len * sizeof(IOUint));
                printf("io count = %u\n", ioVector->len);
            }
        } else if (strncmp(line, "[\"io", 4) == 0) {
            printf("input io array:\n");
        } else if (line[0] == '[' && line[1] > '0' && line[1] <= '9') {
            IOUint io;
            sscanf(line, "[%u,%u,%u,%u]", &io.id, &io.wrap, &io.startLpos, &io.endLpos);
            ioArray[ioCount] = io;
            ioCount++;
            printf("io [%u] : %s", ioCount, line);
        }
    }
    printf("\n\n");

    fclose(file);

    if (ioVector->len != ioCount) {
        printf("Error! len != ioCount\n");
        return RETURN_ERROR;
    }
    if (ioVector->len > MAX_IO_NUM) {
        printf("Error! IO number(%u) should less than %u\n", ioVector->len, MAX_IO_NUM);
        return RETURN_ERROR;
    }
    ioVector->ioArray = ioArray;

    return RETURN_OK;
}

/* 检查输入参数合法性 */

int main(int argc, char *argv[])
{
    printf("Welcome to HW project.\n");

    /* 输入dataset文件地址 */
    int opt;
    char *file = NULL;
    // 使用 getopt 解析命令行参数
    while ((opt = getopt(argc, argv, "f:")) != -1) {
        switch (opt) {
            case 'f':
                file = optarg;
                break;
            default:
                fprintf(stderr, "Usage: %s -f filename. [example: ./main -f /heme/case_1.txt] \n", argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    if (file == NULL) {
        fprintf(stderr, "Usage: %s -f filename. [example: ./main -f /heme/case_1.txt] \n", argv[0]);
        exit(EXIT_FAILURE);
    }

    printf("The file path is: %s\n", file);

    /* 获取输入参数 */
    InputParam *inputParam = (InputParam *)malloc(sizeof(InputParam));
    // char *dataset_case = "/home/m00498798/contest/dataset/case_1.txt";
    int32_t ret = parseFile(file, &inputParam->headInfo, &inputParam->ioVec);
    if (ret < 0) {
        printf("InputParam error\n");
        return RETURN_ERROR;
    }
    OutputParam *output = (OutputParam *)malloc(sizeof(OutputParam));
    output->len = inputParam->ioVec.len;
    output->sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));

    /* 算法执行 */
    ret = AlgorithmRun(inputParam, output);

    printf("output sequence: [");
    for (uint32_t i = 0; i < output->len; i++) {
        printf("%u, ", output->sequence[i]);
    }
    printf("]\n");

    free(inputParam->ioVec.ioArray);
    free(inputParam);
    free(output->sequence);
    free(output);

    return RETURN_OK;
}