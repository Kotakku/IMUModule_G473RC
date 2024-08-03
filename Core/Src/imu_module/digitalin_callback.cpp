#include "stmbed/digital_in.hpp"

#ifdef HAL_GPIO_MODULE_ENABLED
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    using namespace stmbed;
    callback::callback<DigitalIn::CallbackFnType>(GPIO_Pin);
}
#endif // HAL_GPIO_MODULE_ENABLE