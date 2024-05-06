#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <filters.h>

ZTEST_SUITE(test_filters, NULL, NULL, NULL, NULL, NULL);

ZTEST(test_filters, test_lowpass1st) {
    #include "datas_test_lowpass1st.h"
    LowPassFirstOrderFilter aFilter(1.0, 5.0);

    for (int k=0; k < N_DATAS; k++) {
        float32_t yfilt = aFilter.calculateWithReturn(yref[k]);
        zexpect_within(yfilt, y[k], 1e-5, "pb yfilt=%f y[k] = %f, yref[k] = %f", yfilt, y[k], yref[k]);

    }
}

ZTEST(test_filters, test_notchfilter) {
    #include "datas_test_notch_filter.h"
    NotchFilter aFilter(1e-3, 50.0, 5.0);
    for (int k=0; k < N_DATAS; k++) {
        float32_t yfilt = aFilter.calculateWithReturn(yref[k]);
        zexpect_within(yfilt, y[k], 5e-4, "pb yfilt=%f y[k] = %f, yref[k] = %f", yfilt, y[k], yref[k]);

    }
    
}

ZTEST(test_filters, test_pllangle) {
    #include "pll_data_test.h"
    const float32_t Ts = 100e-6F;
    const float32_t f0 = 50.0F;
    const float32_t w0 = 2.0F * PI * f0;
    const uint32_t N = 100;
    float32_t time;
    float32_t angle = 0.0F;
    PllAngle pll(Ts, f0, 0.02F);
    uint32_t k;
    pll.reset(0.9 * f0);
    for (k=0; k < N; k++) {
        time += Ts;
        angle += w0 * Ts;
        angle = ot_modulo_2pi(angle);
        PllDatas result = pll.calculateWithReturn(angle);
        zexpect_within(w_est[k], result.w, 0.2, "west[k] = %f and result.w = %f", w_est[k], result.w);
    }
}


