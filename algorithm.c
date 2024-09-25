#include "algorithm.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>  // 包含用于 memcpy 的头文件

#define MAX_ITER 10000  // 最大迭代次数
#define INIT_TEMP 10000.0  // 初始温度
#define COOLING_RATE 0.995  // 冷却速率

/*
// 寻址时间计算
uint32_t SeekTimeCalculate(const HeadInfo *start, const HeadInfo *target) {
    int32_t wrap_diff = abs((int32_t)(start->wrap - target->wrap));
    int32_t lpos_diff = abs((int32_t)(start->lpos - target->lpos));
    return wrap_diff * 1000 + lpos_diff;  // 假设每wrap的转换耗时1000ms，每lpos耗时1ms
}
*/
// 计算调度序列的总寻址时间
double CalculateTotalCost(const InputParam *input, const uint32_t *sequence) {
    double totalCost = 0;
    HeadInfo currentHead = input->headInfo;

    for (uint32_t i = 0; i < input->ioVec.len; i++) {
        IOUint *io = &input->ioVec.ioArray[sequence[i]];
        HeadInfo target = {io->wrap, io->startLpos, HEAD_RW};

        // 只计算寻址时间
        uint32_t seekTime = SeekTimeCalculate(&currentHead, &target);
        totalCost += seekTime;

        // 更新磁头位置
        currentHead.wrap = io->wrap;
        currentHead.lpos = io->endLpos;
    }

    return totalCost;
}


// 贪心算法：选择最近的IO请求
void GreedySchedule(const InputParam *input, OutputParam *output) {
    uint32_t remaining = input->ioVec.len;
    int *visited = (int *)calloc(remaining, sizeof(int));
    
    HeadInfo currentHead = input->headInfo;

    for (uint32_t i = 0; i < output->len; i++) {
        int nearestIndex = -1;
        uint32_t nearestTime = UINT32_MAX;

        // 找到最近的IO请求
        for (uint32_t j = 0; j < input->ioVec.len; j++) {
            if (visited[j]) continue;

            IOUint *io = &input->ioVec.ioArray[j];
            HeadInfo target = {io->wrap, io->startLpos, HEAD_RW};
            uint32_t seekTime = SeekTimeCalculate(&currentHead, &target);

            if (seekTime < nearestTime) {
                nearestTime = seekTime;
                nearestIndex = j;
            }
        }

        if (nearestIndex != -1) {
            visited[nearestIndex] = 1;
            output->sequence[i] = input->ioVec.ioArray[nearestIndex].id;

            // 更新磁头位置
            currentHead.wrap = input->ioVec.ioArray[nearestIndex].wrap;
            currentHead.lpos = input->ioVec.ioArray[nearestIndex].endLpos;
        }
    }

    free(visited);
}

void SimulatedAnnealing(const InputParam *input, OutputParam *output) {
    // 确保 output->sequence 已经被初始化
    if (output->sequence == NULL) {
        output->sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));
    }

    uint32_t *currentSequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));
    uint32_t *bestSequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));

    // 使用贪心算法生成的初始顺序
    memcpy(currentSequence, output->sequence, output->len * sizeof(uint32_t));

    // 记录当前最佳序列
    memcpy(bestSequence, currentSequence, output->len * sizeof(uint32_t));

    // 使用TotalAccessTime计算初始序列的访问时间
    AccessTime accessTimeBest;
    // 传递 AccessTime 结构体中的具体字段
    TotalAccessTime(input, output, &accessTimeBest.addressDuration, &accessTimeBest.readDuration);
    double bestCost = accessTimeBest.addressDuration + accessTimeBest.readDuration;

    // 初始温度和冷却速率
    double temp = INIT_TEMP;
    double minTemp = 1e-3;  // 设置最低温度，防止过度冷却

    for (int iter = 0; iter < MAX_ITER && temp > minTemp; iter++) {
        // 分块逆序操作：随机选择一个块并对该块进行逆序处理
        uint32_t newSequence[output->len];
        memcpy(newSequence, currentSequence, output->len * sizeof(uint32_t));

        // 随机选择一个块的起始位置和大小
        uint32_t blockSize = (rand() % 50) + 10;  // 块大小在10到50之间
        uint32_t startIdx = rand() % (output->len - blockSize);

        // 逆序该块
        uint32_t endIdx = startIdx + blockSize - 1;
        while (startIdx < endIdx) {
            uint32_t tempVal = newSequence[startIdx];
            newSequence[startIdx] = newSequence[endIdx];
            newSequence[endIdx] = tempVal;
            startIdx++;
            endIdx--;
        }

        // 用 TotalAccessTime 计算新序列的访问时间
        AccessTime accessTimeNew;
        // 使用 output 而不是 newOutput，确保已初始化的结构体传递给 TotalAccessTime
        TotalAccessTime(input, output, &accessTimeNew.addressDuration, &accessTimeNew.readDuration);
        double newCost = accessTimeNew.addressDuration + accessTimeNew.readDuration;

        // 计算接受新序列的概率
        double deltaCost = (bestCost - newCost) / temp;
        if (deltaCost > 100) deltaCost = 100;  // 避免数值过大
        if (deltaCost < -100) deltaCost = -100;  // 避免数值过小
        double acceptanceProbability = exp(deltaCost);

        // 如果新解更优或以一定概率接受新解
        if (newCost < bestCost || ((double)rand() / RAND_MAX) < acceptanceProbability) {
            memcpy(currentSequence, newSequence, output->len * sizeof(uint32_t));
            if (newCost < bestCost) {
                bestCost = newCost;
                memcpy(bestSequence, newSequence, output->len * sizeof(uint32_t));
            }
        }

        // 动态调整冷却速率：自适应调整冷却速率
        if (temp > 1.0) {
            temp *= COOLING_RATE;  // 高温时按标准速率冷却
        } else {
            temp *= 0.99;  // 低温时加速冷却
        }
    }

    // 记录最终最优解
    memcpy(output->sequence, bestSequence, output->len * sizeof(uint32_t));

    // 释放动态分配的内存
    free(currentSequence);
    free(bestSequence);
}

// 主算法接口：结合贪心和模拟退火的混合算法
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output) {
    if (input == NULL || output == NULL) {
        return RETURN_ERROR;  // 参数无效时返回错误
    }

    // 初始化 output 的长度为输入的 ioVec 长度
    output->len = input->ioVec.len;

    // 分配 output->sequence 数组的内存，如果尚未分配
    if (output->sequence == NULL) {
        output->sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));
        if (output->sequence == NULL) {
            return RETURN_ERROR;  // 内存分配失败时返回错误
        }
    }

    // 先使用贪心算法生成初始调度顺序
    GreedySchedule(input, output);

    // 使用模拟退火算法在贪心基础上进一步优化
    SimulatedAnnealing(input, output);

    return RETURN_OK;  // 成功执行后返回OK
}


void PrintMetrics(const KeyMetrics *metrics)
{
    printf("\nKey Metrics:\n");
    printf("\talgorithmRunningDuration:\t %f ms\n", metrics->algorithmRunningDuration);
    printf("\taddressingDuration:\t\t %u ms\n", metrics->addressingDuration);
    printf("\treadDuration:\t\t\t %u ms\n", metrics->readDuration);
    printf("\ttapeBeltWear:\t\t\t %u\n", metrics->tapeBeltWear);
    printf("\ttapeMotorWear:\t\t\t %u\n", metrics->tapeMotorWear);
}

/**
 * @brief  算法运行的主入口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return uint32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
uint32_t AlgorithmRun(const InputParam *input, OutputParam *output)
{
    int32_t ret;
    
    KeyMetrics metrics = {
	    .algorithmRunningDuration = 0.0,
	    .addressingDuration = 0,
	    .readDuration = 0,
	    .tapeBeltWear = 0,
	    .lposPassTime = {0},  // 数组字段需要单独初始化
	    .tapeMotorWear = 0
	};
	ret = RETURN_OK;  // 初始化 ret 为成功状态
	
	ret = IOScheduleAlgorithm(input, output);
	if (ret != RETURN_OK) {  // 检查函数是否成功执行
	    printf("Error occurred during I/O scheduling\n");
	    return RETURN_ERROR;  // 如果出错，返回错误码
	}
    struct timeval start, end;
    long seconds, useconds;
    // 记录开始时间
    gettimeofday(&start, NULL);

    ret = IOScheduleAlgorithm(input, output);

    // 记录结束时间
    gettimeofday(&end, NULL);

    // 计算秒数和微秒数
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;

    // 总微秒数
    metrics.algorithmRunningDuration = ((seconds)*1000000 + useconds);

    /* 访问时间 */
    TotalAccessTime(input, output, &metrics.addressingDuration, &metrics.readDuration);

    /* 带体磨损 */
    metrics.tapeBeltWear = TotalTapeBeltWearTimes(input, output, metrics.lposPassTime);

    /* 电机磨损 */
    metrics.tapeMotorWear = TotalMotorWearTimes(input, output);

    PrintMetrics(&metrics);

    return RETURN_OK;
}
