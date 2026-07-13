#ifndef MODULE_PROP_H
#define MODULE_PROP_H

#include "Global_Define.h"


#define VERT_PROP_NUM   4
#define HORIZ_PROP_NUM  4
#define TOTAL_PROP_NUM  (VERT_PROP_NUM + HORIZ_PROP_NUM)

typedef enum {
    V_IDX_0 = 0,
    V_IDX_1,
    V_IDX_2,
    V_IDX_3,
} Vertical_Idx_t;

typedef enum {
    H_IDX_0 = 0,
    H_IDX_1,
    H_IDX_2,
    H_IDX_3,
} Horizontal_Idx_t;

typedef enum {
    CTRL_MODE_STOP   = 0,
    CTRL_MODE_MANUAL = 1,
    CTRL_MODE_AUTO   = 2,
} Thruster_CtrlMode_t;

typedef struct
{
    bool                 Deepth_Flag;       /* 定深标志 */
    bool                 Heading_Flag;      /* 定航向标志 */
    uint16_t             Roll_Change;       /* 横滚角度调节量 */
    uint8_t              Vertical_Value_DEC;    /* 垂向下潜运动值 */
	  uint8_t              Vertical_Value_ADD;    /* 垂向上浮运动值 */
    uint16_t             Manual_V_CQ[VERT_PROP_NUM];   /* 手动垂向控制量 [逻辑索引] */
	  uint16_t             Manual_V_DIR[VERT_PROP_NUM];
	  uint8_t              Manual_V_RES[VERT_PROP_NUM];
    uint16_t             Manual_H_CQ[HORIZ_PROP_NUM];   /* 手动水平控制量 [逻辑索引] */
		uint16_t             Manual_H_DIR[HORIZ_PROP_NUM];
	  uint8_t              Manual_H_RES[HORIZ_PROP_NUM];
    uint16_t             Auto_V_CQ[VERT_PROP_NUM];      /* 自动垂向控制量 [逻辑索引] */
    uint16_t             Auto_H_CQ[HORIZ_PROP_NUM];     /* 自动水平控制量 [逻辑索引] */
    uint16_t             Final_CQ[TOTAL_PROP_NUM];      /* 最终输出值 [逻辑索引] */
} Prop_Control_Data_TypeDef;

extern Prop_Control_Data_TypeDef Prop_Ctrol;

int16_t  Turn_Conv_CQ(uint16_t cq);
uint16_t Clamp(uint16_t val, uint16_t min, uint16_t max);
void     Prop_Init(void);
void     Prop_Apply_Vertical(void);
void     Prop_Apply_Horizontal(void);

void     Manual_V_Ctrl(void *argument);
void     Manual_H_Ctrl(void *argument);
void     Auto_V_Ctrl(void *argument);
void     Auto_H_Ctrl(void *argument);

#endif

