/* NOTE: Definitions used internal to LLL implementations */

int lll_prepare_done(void *param);
int lll_done(void *param);
bool lll_is_done(void *param);
int lll_clk_on(void);
int lll_clk_on_wait(void);
int lll_clk_off(void);

