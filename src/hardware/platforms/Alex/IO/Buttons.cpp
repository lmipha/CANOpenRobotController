#include "Buttons.h"

#include <fstream>
#include <iostream>

#include "iobb.h"

Buttons::Buttons() {
    spdlog::debug("Button object created");
    iolib_init();
    iolib_setdir(8, 10, BBBIO_DIR_IN);

    errorButton = false;
};
Buttons::~Buttons() {
    // Todo: Check if destructor is necessary
};
void Buttons::updateInput() {
    errorButton = is_high(8, 10);

    // std::cout << "Error button value: " << errorButton << std::endl;
};

bool Buttons::getErrorButton() {
    return errorButton;
};