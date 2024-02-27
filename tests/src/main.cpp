#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
// #include <zephyr/console/console.h>
#include <trigo.h>
#include <rst.h>
#include <zephyr/logging/log.h>
#include <pid.h>
#include <pr.h>
#include <filters.h>

LOG_MODULE_REGISTER(test_control, LOG_LEVEL_ERR);

ZTEST_SUITE(trigo, NULL, NULL, NULL, NULL, NULL);

ZTEST(trigo, test_sin)
{
    #include "datas_test_trigo.h"
    float32_t angle;
    float32_t sin_data;
    float32_t ot_sin_data;
    float32_t delta_sin;
    for (uint8_t k = 0; k< ARRAY_SIZE; k++) {
        angle = ((float32_t *)random_angles)[k];
        sin_data = ((float32_t *)random_sin)[k];
        ot_sin_data = ot_sin(angle);
        delta_sin = sin_data - ot_sin_data;
        zexpect_between_inclusive(delta_sin, -2e-5, 2e-5, "sin(%f) !=  %f: %.6f", angle, ot_sin_data, delta_sin);
    }

}

ZTEST(trigo, test_cos)
{
    #include "datas_test_trigo.h"
    float32_t angle;
    float32_t cos_data;
    float32_t ot_cos_data;
    float32_t delta_cos;
    for (uint8_t k = 0; k< ARRAY_SIZE; k++) {
        angle = ((float32_t *)random_angles)[k];
        cos_data = ((float32_t *)random_cos)[k];
        ot_cos_data = ot_cos(angle);
        delta_cos = cos_data - ot_cos_data;
        zexpect_between_inclusive(delta_cos, -2e-5, 2e-5, "cos(%f) !=  %f: %.6f", angle, ot_cos_data, delta_cos);
    }

}

ZTEST(trigo, test_modulo_2pi) {
    #include "datas_test_trigo.h"
    float32_t angle;
    float32_t modulo_data;
    float32_t ot_modulo;
    float32_t delta;
    for (uint8_t k = 0; k< ARRAY_SIZE; k++) {
        angle = ((float32_t *)random_angles)[k];
        modulo_data = ((float32_t *)random_modulo_2pi)[k];
        ot_modulo = ot_modulo_2pi(angle);
        delta = ot_modulo - modulo_data;
        zassert_between_inclusive(delta, -2e-5, 2e-5, "error delta = %f", delta);
    }
}

ZTEST_SUITE(rst, NULL, NULL, NULL, NULL, NULL);

ZTEST(rst, test_fir_update) {
    float32_t value;
    Fir myFir = Fir();
    const float32_t c[4] = {0.25, 0.25, 0.25, 0.25};
    myFir.init(4, c);
    value = myFir.update(1.0);
    zexpect_equal(value, 0.25, "retvalue = %f", value);
    value = myFir.update(1.0);
    zexpect_equal(value, 0.5, "retvalue = %f", value);
    value = myFir.update(1.0);
    zexpect_equal(value, 0.75, "retvalue = %f", value);
    value = myFir.update(1.0);
    zexpect_equal(value, 1.0, "retvalue = %f", value);
    value = myFir.update(1.0);
    zexpect_equal(value, 1.0, "retvalue = %f", value);
}

ZTEST(rst, test_rst_init) {
    RST my_rst;
    const float r[1] = {1.0};
    float s[2] = {1.0, 2.0};
    const float t[1] = {1.0};
    RstParams p(0.1, 1, r, 2, s, 1, t, 0.0, 1.0); 
    int8_t is_ok = my_rst.init(p);
    zexpect_true(is_ok == 0, "init problem");

    float bad_s[2] = {0.0, 2.0};
    p.s = bad_s;
    is_ok = my_rst.init(p);
    zexpect_true(is_ok < 0.0, "init problem");
}

ZTEST(rst, test_rst_update) {
    #include "datas_test_rst.h"
    RST my_rst = RST();
    const uint8_t nr = 3;
    const float R[] = { 0.8914, -1.1521, 0.3732 };
    const uint8_t ns = 6;
    const float S[] = { 0.2, 0.0852, -0.0134, -0.0045, -0.1785, -0.0888 };
    const uint8_t nt = 3;
    const float T[] = { 1.0, -1.3741, 0.4867 };
    RstParams p(5, nr, R, ns, S, nt, T, -5.0, 5.0);
    my_rst.init(p);
    float32_t u;

    for (uint8_t step=0; step < 20; step++)
    {
        u = my_rst.calculateWithReturn(y_ref[step], y_meas[step]);
        zexpect_between_inclusive(u-u_test[step], -0.05, 0.05, "%i, u = %f, u_test = %f", step, u, u_test[step]);
    }

}

ZTEST(rst, test_rst_limit) {
    zassert_ok(false, "rst_limit to implement");
}

ZTEST(rst, test_fir_setcoeff) {
    float32_t value;
    Fir myFir = Fir();
    const float32_t c[4] = {0.25, 0.25, 0.25, 0.25};
    myFir.init(4, c);
    value = myFir.update(1.0);
    zexpect_equal(value, 0.25, "retvalue = %f", value);
    myFir.setCoeff(1, 0.0);
    myFir.setCoeff(2, 0.0);
    myFir.setCoeff(3, 0.0);
    value = myFir.update(1.0);
    zexpect_equal(value, 0.25, "retvalue = %f", value);
    value = myFir.update(1.0);
    zexpect_equal(value, 0.25, "retvalue = %f", value);
}

// PidStandard
struct pid_fixture_t {
    PidParams params;
};

static void *pid_test_setup(void)
{
    static struct pid_fixture_t pid_fixture;
    return &pid_fixture;
}

static void pid_test_before(void *f) {
    pid_fixture_t *pid_fixture = (pid_fixture_t *)f;
    pid_fixture->params.Ts = 5.0;
    pid_fixture->params.Kp = 0.73;
    pid_fixture->params.Ti = 2.735;
    pid_fixture->params.Td = 0.122;
    pid_fixture->params.N = 10.0;
    pid_fixture->params.lower_bound = -0.8;
    pid_fixture->params.upper_bound = 0.8;
}

ZTEST_SUITE(test_pid, NULL, pid_test_setup, pid_test_before, NULL, NULL);

ZTEST_F(test_pid, test_init) {
    Pid pid;
    pid_fixture_t *pid_fixture = (pid_fixture_t *)fixture;
    pid.init(pid_fixture->params);
}

ZTEST_F(test_pid, test_calculate) {
    Pid pid;
    pid_fixture_t *pid_fixture = (pid_fixture_t *)fixture;
    pid.init(pid_fixture->params);
    // data with pid saturation activate
    #include "datas_test_pid_standard.h"
    int n = sizeof(yref) / sizeof(yref[0]);
    for (int k=0; k < n-1; k++)
    {
        float32_t out = pid.calculateWithReturn(yref[k], y[k]);
        zexpect_within(u[k+1], out, 5e-6, "k=%d u[k] = %f, pid u = %f", k, u[k+1], out);
    }

}

ZTEST(test_pid, test_null_derivative) {
    float32_t Ts = 5.0;
    float32_t Kp = 0.73;
    float32_t Ti = 2.735;
    float32_t Td = 0.0;
    float32_t N = 0.0;
    float32_t lower_bound = -0.8;
    float32_t upper_bound = 0.8;
    PidParams params(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
    Pid pid;
    pid.init(params);
    // data with pid saturation activate
    for (int k=0; k < 10; k++)
    {
        float32_t out = pid.calculateWithReturn(0.1, 0.0);
        bool test = ((*(uint32_t *)&out) >> 20) == 0x7F8;
        zexpect_false(test, "result is inf, %x", *(uint32_t *) &out);
    }

}

ZTEST_F(test_pid, test_bad_init_values) {
    pid_fixture_t *pid_fixture = (pid_fixture_t *)fixture;
    PidParams params = pid_fixture->params;
    //PidParams params(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
    Pid pid;
    int8_t is_ok; 

    params.Ts = -1.0;
    is_ok = pid.init(params);
    zexpect_true(0 > is_ok, "with Ts<0, it should return -EINVAL");

    params.Ts = 5.0;
    params.Ti = 0.0;
    is_ok = pid.init(params);
    zexpect_true (0 > is_ok, "with Ti==0, it should return -EINVAL");

    params.Ti = 2.735;
    params.Kp = 0.0;
    is_ok = pid.init(params);
    zexpect_true (0 > is_ok, "with Kp==0, it should return -EINVAL");

    params.Kp = 0.73;
    params.lower_bound = 1.0;
    params.upper_bound = -1.0;
    is_ok = pid.init(params);
    zexpect_true (0 > is_ok, "with incorrect bounds  it should return -EINVAL");
}

ZTEST_F(test_pid, test_reset) {
    pid_fixture_t *pid_fixture =  (pid_fixture_t *)fixture;
    Pid pid;
    pid.init(pid_fixture->params);
    pid.reset(-0.4);
    float32_t value = pid.calculateWithReturn(1.0, 1.0);
    zexpect_within(-0.4, value, 1e-7, "value = %f", value);
}

// Pr
ZTEST_SUITE(test_pr, NULL, NULL, NULL, NULL, NULL);


ZTEST(test_pr, test_calculate) {
    float32_t Ts = 9.999999747378752e-05;
    float32_t Kp = 0.20000000298023224;
    float32_t Kr = 300.0;
    float32_t w = 2513.274169921875;
    float32_t phi = 0.3769911229610443;
    float32_t lower_bound = -1.0;
    float32_t upper_bound = 1.0;
    PrParams params(Ts, Kp, Kr, w, phi, lower_bound, upper_bound);
    Pr pr;
    pr.init(params);
    // data with pid saturation activate
    #include "data_test_pr.h"
    int n = sizeof(yref) / sizeof(yref[0]);
    for (int k=0; k < n-1; k++)
    {
        float32_t out = pr.calculateWithReturn(yref[k], y_nosat[k]);
        zexpect_within(u_nosat[k+1], out, 3e-4, "k=%d u[k] = %f, pid u = %f", k, u_nosat[k+1], out);
    }

}

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

