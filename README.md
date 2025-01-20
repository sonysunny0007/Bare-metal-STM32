# Bare-metal-STM32

## Bare-Metal Programming

This repository is dedicated to exploring **bare-metal programming** on embedded systems. It provides practical examples and guides to interact directly with hardware without relying on an operating system.

## Getting Started

### Introduction

Bare-metal programming is a low-level programming approach where software interacts directly with hardware without any intermediate operating system or framework. This repository contains step-by-step examples to help you get started with embedded development.

### Current Example: Blinky for Nucleo-F429ZI

The repository currently includes a **blinky example** for the **Nucleo-F429ZI** board. The code demonstrates how to toggle an LED using basic GPIO configuration.

#### Why Bare-Metal Programming?

- **Efficient Resource Use**: Direct access to hardware allows for optimized performance.
- **Learning Experience**: Gain deeper insights into microcontroller architectures and peripherals.
- **No OS Overhead**: Write code without the complexity and limitations of an operating system.

## Prerequisites

To run the examples, you need:
1. **Toolchain**: ARM GCC or other appropriate compiler for your platform.
2. **IDE**: You can use an IDE like VSCode, Eclipse, or simply a text editor.
3. **Debugger/Flasher**: ST-Link or similar hardware for programming the microcontroller.

## How to Set Up the Environment

1. **Install ARM GCC Toolchain**: Follow the instructions on [ARM GCC Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gcc) to install the compiler.
   
2. **Set Up Your IDE**: You can choose your preferred development environment, such as:
   - [VSCode](https://code.visualstudio.com/) with the Cortex-Debug extension.
   - [Eclipse](https://www.eclipse.org/downloads/) for embedded development.

3. **Connect the Board**: Make sure you have the **Nucleo-F429ZI** board connected via ST-Link or other compatible flashing tools.

4. **Compile and Flash**: Build the project and flash it onto the microcontroller using your chosen toolchain and debugger.

## How to Use the Examples

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/sonysunny0007/Bare-metal-STM32.git
   cd Bare-metal-STM32
