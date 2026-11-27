#pragma once

bool setupTimer(TIM_TypeDef *t) {
    t->CR1 = t->CR2 = t->DIER = t->CCER = t->PSC = 0;
    t->ARR = 65535; t->CCMR1 = 1U|(8U<<4)|(1U<<8)|(8U<<12);
    t->SMCR = 3; t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    t->EGR = TIM_EGR_UG; t->CNT = t->SR = 0; t->CR1 = TIM_CR1_CEN;
    return (t->CR1 & 1) && (t->SMCR & 7) == 3 && t->ARR == 65535 && !t->DIER;
}
bool initEncoders() {
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
    __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
    const uint32_t remap = AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP;
    AFIO->MAPR = (AFIO->MAPR & ~(remap | AFIO_MAPR_SWJ_CFG)) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    pinMode(PA6,INPUT_PULLUP); pinMode(PA7,INPUT_PULLUP);
    pinMode(PA0,INPUT_PULLUP); pinMode(PA1,INPUT_PULLUP);
    return !((remap|AFIO_MAPR_TIM4_REMAP) & AFIO->MAPR) && setupTimer(TIM3) && setupTimer(TIM2);
}
bool readRegs(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t n) {
    bus.beginTransmission(addr); bus.write(reg);
    if (bus.endTransmission(false) || bus.requestFrom(addr,n) != n) return false;
    for (int i=0; i<n; ++i) { int b = bus.read(); if (b<0) return false; data[i]=uint8_t(b); }
    return true;
}
bool writeReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t value) {
    bus.beginTransmission(addr); bus.write(reg); bus.write(value);
    return bus.endTransmission() == 0;
}
bool initImu() {
    for (mpu=0x68; mpu<=0x69; ++mpu) {
        uint8_t who = 0;
        if (readRegs(Wire,mpu,0x75,&who,1) && who==0x68)
            return writeReg(Wire,mpu,0x6b,1) && writeReg(Wire,mpu,0x6c,0) &&
                   writeReg(Wire,mpu,0x1a,3) && writeReg(Wire,mpu,0x19,9) &&
                   writeReg(Wire,mpu,0x1b,0);
    }
    return false;
}
