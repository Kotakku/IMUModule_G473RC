#include "main.h"
#include "spi_slave.hpp"
#include "stmbed/callbacks/callback.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    using namespace stmbed;
    callback::callback<SPISlave::DMACallbackFnType>(reinterpret_cast<intptr_t>(hspi));
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    using namespace stmbed;
    callback::callback<SPISlave::DMACallbackFnType>(reinterpret_cast<intptr_t>(hspi));
}

#endif // HAL_SPI_MODULE_ENABLED
