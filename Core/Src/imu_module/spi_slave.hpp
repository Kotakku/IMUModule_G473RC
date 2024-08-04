#pragma once

#include "main.h"
#include "stmbed/callbacks/callback.hpp"

#include <stdio.h>

class SPISlave {
public:
    using DMACallbackFnType = void(void);

    SPISlave(SPI_HandleTypeDef *hspi) : hspi_(hspi) {
        bool has_spi_dmatx = hspi_->hdmatx != nullptr;
        bool has_spi_dmarx = hspi_->hdmarx != nullptr;

        if (has_spi_dmatx && has_spi_dmarx) {
            is_dma_enabled_ = true;

            std::function<void(void)> dma_callback = std::bind(&SPISlave::dma_callback, this);
            stmbed::callback::attach(reinterpret_cast<intptr_t>(hspi_), std::move(dma_callback), 100);
        }
    }

    bool available() { return HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_READY; }
    bool available_dma() { return spi_state_ == SPI_COMPLETE; }

    void on_dma_complete(std::function<void(void)> callback) {
        stmbed::callback::attach(reinterpret_cast<intptr_t>(hspi_), std::move(callback), 100);
    }

    bool write_read(uint8_t *tx_data, uint8_t *rx_data, size_t size, uint32_t timeout) {
        if (is_dma_enabled_) {
            spi_state_ = SPI_WAIT;
            // HAL_SPI_DMAStop(hspi_);
            return HAL_SPI_TransmitReceive_DMA(hspi_, tx_data, rx_data, size) == HAL_OK;
        } else {
            return HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, size, timeout) == HAL_OK;
        }
    }

    SPI_HandleTypeDef *get_handle() const { return hspi_; }

    bool is_dma_enabled() const { return is_dma_enabled_; }

private:
    SPI_HandleTypeDef *hspi_;
    bool is_dma_enabled_ = false;
    enum SPIState { SPI_WAIT, SPI_COMPLETE, SPI_ERROR };

    SPIState spi_state_ = SPI_COMPLETE;

    void dma_callback(void) {
        spi_state_ = SPI_COMPLETE;
    }
};
