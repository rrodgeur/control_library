#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <transform.h>

LOG_MODULE_DECLARE(test_control);

ZTEST_SUITE(test_transform, NULL, NULL, NULL, NULL, NULL);

ZTEST(test_transform, test_clarke) {
    // 0°
    three_phase_t* Xabc_tests = new three_phase_t[3];
    Xabc_tests[0] = three_phase_t(1.0F, -0.5F, -0.5F);
    Xabc_tests[1] = three_phase_t(0.0F, 0.866025F, -0.866025F);
    Xabc_tests[2] = three_phase_t(.707107F, 0.258819F, -0.965926F);
    clarke_t *Xabo_results = new clarke_t[3];
    Xabo_results[0] = clarke_t(1.0F, 0.0F, 0.0F);
    Xabo_results[1] = clarke_t(0.0F, 1.0F, .0F);
    Xabo_results[2] = clarke_t(.707107F, 0.707107F, 0.0F);

    for (int k=0; k<3;k++) {
        clarke_t Xabo = Transform::clarke(Xabc_tests[k]);
        zexpect_within(Xabo.alpha, Xabo_results[k].alpha, 1e-4, "Xabo.alpha=%f", Xabo_results[k].alpha);
        zexpect_within(Xabo.beta, Xabo_results[k].beta, 1e-4, "Xabo.beta=%f", Xabo_results[k].beta);
        zexpect_within(Xabo.o, Xabo_results[k].o, 1e-4, "Xabo.o = %f", Xabo_results[k].o);
        three_phase_t Xabc_t = Transform::clarke_inverse(Xabo);
        zexpect_within(Xabc_t.a, Xabc_tests[k].a, 1e-4, "Xabc.a=%f", Xabc_tests[k].a);
        zexpect_within(Xabc_t.b, Xabc_tests[k].b, 1e-4, "Xabc.b=%f", Xabc_tests[k].b);
        zexpect_within(Xabc_t.c, Xabc_tests[k].c, 1e-4, "Xabc.c = %f", Xabc_tests[k].c);

    }
}

ZTEST(test_transform, test_dqo) {
    // 0°
    three_phase_t* Xabc_tests = new three_phase_t[3];
    float *angles = new float[3];
    angles[0] = 0.0;
    angles[1] = PI/2.0F;
    angles[2] = PI/4.0F;
    dqo_t *Xdqo_results = new dqo_t[3];
    Xdqo_results[0] = dqo_t(1.0F, 0.0F, 0.0F);
    Xdqo_results[1] = dqo_t(1.0F, 0.0F, 0.0F);
    Xdqo_results[2] = dqo_t(1.0F, 0.0F, 0.0F);

    for (int k=0; k<3;k++) {
        Xabc_tests[k] = three_phase_t(ot_cos(angles[k]), ot_cos(angles[k]-2.0F*PI/3.0F), ot_cos(angles[k]-4.0F*PI/3.0F));
        dqo_t Xdqo = Transform::to_dqo(Xabc_tests[k], angles[k]);
        zexpect_within(Xdqo.d, Xdqo_results[k].d, 2e-5, "error d =%f", -Xdqo_results[k].d + Xdqo.d);
        zexpect_within(Xdqo.q, Xdqo_results[k].q, 1e-5, "error q=%f", -Xdqo_results[k].q + Xdqo.q);
        zexpect_within(Xdqo.o, Xdqo_results[k].o, 1e-5, "error o = %f", -Xdqo_results[k].o + Xdqo.o);
        three_phase_t Xabc_t = Transform::to_threephase(Xdqo, angles[k]);
        zexpect_within(Xabc_t.a, Xabc_tests[k].a, 2e-5, "Xabc.a=%f", Xabc_tests[k].a);
        zexpect_within(Xabc_t.b, Xabc_tests[k].b, 1e-5, "Xabc.b=%f", Xabc_tests[k].b);
        zexpect_within(Xabc_t.c, Xabc_tests[k].c, 1e-5, "Xabc.c = %f", Xabc_tests[k].c);

    }
}

