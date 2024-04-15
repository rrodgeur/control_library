# Getting started

The control library is in the `src/` directory.
It has been written in c++. It has mainly been developped to be used with 
The [OwnTech Power API](https://github.com/owntech-foundation/Core) based on [Zephyr](https://www.zephyrproject.org/)
and integrated with [PlatformIO](https://platformio.org/).

Here we show step-by-step an example on how to use a `Pid()` assuming the installation 
step is done [link](link).


The use of the `Pid` is based on **3 steps**.

1. Pid object instanciation (declaration).
2. Pid initialisation.
3. Pid execution.


## Pid object instanciation.

For each `controller` like (`Pid`, `Rst`, `Pr`) we have to define a parameter structure.


```c++
static float32_t Ti = 7.5175e-5F;
static float32_t Td = 0.0F;
static float32_t N = 0.0F;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = 100.0e-6F;
```
```c++
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;
```
## Pid initialization.

## Pid execution.


