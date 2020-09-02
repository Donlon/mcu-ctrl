#ifndef __speed_H
#define __speed_H
#ifdef __cplusplus
extern "C" {
#endif

void SpeedMesurement_Init(void);

void CalculateSpeed(void);

typedef struct {
    int32_t LeftDistance;
    int32_t RightDistance;

} SpeedDataTypeDef;

#ifdef __cplusplus
}
#endif
#endif /*__speed_H*/
