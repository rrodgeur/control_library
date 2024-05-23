#include "factory.h"

Pid ControlFactory::pid(float32_t Ts, float32_t Kp, float32_t Ti, float32_t Td, float32_t N, float32_t lower_bound, float32_t upper_bound) {
    PidParams p(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
    Pid controller = Pid();
    controller.init(p);
    return controller;
}

Pr ControlFactory::pr(float32_t Ts, float32_t Kp, float32_t Kr, float32_t w0, float32_t phi_prime, float32_t lower_bound, float32_t upper_bound){
    PrParams p(Ts, Kp, Kr, w0, phi_prime, lower_bound, upper_bound);
    Pr controller = Pr();
    controller.init(p);
    return controller;
}

RST ControlFactory::rst(float32_t Ts, uint8_t nr, const float32_t *r, uint8_t ns, const float32_t *s, uint8_t nt, const float32_t *t, float32_t lower_bound, float32_t upper_bound){
    RstParams p(Ts, nr, r, ns, s, nt, t, lower_bound, upper_bound);
    RST controller = RST();
    controller.init(p);
    return controller;
}

PllSinus ControlFactory::pllSinus(float32_t Ts, float32_t amplitude, float32_t f0, float32_t rise_time){
    PllSinus pll = PllSinus(Ts, amplitude, f0, rise_time);
    return pll;
}

NotchFilter ControlFactory::notchfilter(float32_t Ts, float32_t f0, float32_t bandwidth){
    NotchFilter filter = NotchFilter(Ts, f0, bandwidth);
    filter.reset();
    return filter;
}

LowPassFirstOrderFilter  ControlFactory::lowpassfilter(float32_t Ts, float32_t tau){
    LowPassFirstOrderFilter filter = LowPassFirstOrderFilter(Ts, tau);
    filter.reset();
    return filter;
}

ControlFactory controlLibFactory = ControlFactory();

