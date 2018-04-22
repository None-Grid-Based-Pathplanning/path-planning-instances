import matplotlib.pyplot as plt
import numpy as np

ax = plt.subplot(111)
t1 = np.arange(1, 13, 1)

pso_bhg_sr = [0.984, 0.892, 0.986, 0.976, 0.984, 0.97, 0.892, 0.894, 0.934, 0.86, 0.628, 0.978]
pso_uhg_sr = [0.946, 0.546, 0.926, 0.914, 0.912, 0.948, 0.812, 0.854,0.936, 0.856,0.634, 0.958]
pso_bhg_ml = [61.23, 56.47, 68.69, 68.00, 63.01, 55.73, 58.33, 57.33, 97.70, 53.83, 56.98, 57.20]
pso_uhg_ml = [60.33, 47.37, 67.62, 61.67, 61.17, 55.25, 55.89, 56.64,133.68, 53.93, 58.11, 57.61]

abc_bhg_sr = [0.998, 0.998, 0.45, 1, 1, 1, 0.892, 0.972, 0.952, 0.944, 0.682, 0.942]
abc_uhg_sr = [0.996, 0.932, 0.15, 0.988, 1, 1, 0.826, 0.938, 0.932, 0.918, 0.644, 0.911]
abc_bhg_ml = [103.66,107.07,78.90,100.64,96.20,105.60,101.47,93.39,99.18,98.25,113.33,111.67]
abc_uhg_ml = [135.78,131.81,97.2,153.72,138.55,134.53,131.96,140.23,138.95,133.27,134.02,130.15]


apf_sds_sr = []
apf_ads_sr = []
apf_sds_ml = []
apf_ads_ml = []

# plt.plot(t1, pso_uhg_ml, 'o-',label="%s" % ("PSO_UHG",), color = "r")
# plt.plot(t1, pso_bhg_ml, '^-',label="%s" % ("PSO_BHG",), color = "b")
plt.plot(t1, abc_uhg_ml, 'o-',label="%s" % ("ABC_UHG",), color = "r")
plt.plot(t1, abc_bhg_ml, '^-',label="%s" % ("ABC_BHG",), color = "b")

#plt.plot(t1, abc_sds_ml,'o-', label="n=%s" % ("ABC_SDS",))
#plt.plot(t1, abc_ads_ml, '^-',label="n=%s" % ("ABC_ADS",))

#plt.plot(t1, apf_sds_ml,'o-', label="n=%s" % ("APF_SDS",))
#plt.plot(t1, apf_ads_ml,'*-', label="n=%s" % ("APF_ADS",))

#leg = plt.legend(loc='title', ncol=2, mode="expand", shadow=False, fancybox=True)
leg = plt.legend(shadow=False, fancybox=True, fontsize = 15)
leg.get_frame().set_alpha(0.5)
plt.xlim((0, 15))
plt.ylim((40, 190))
# plt.xlabel('Workspaces index',fontsize = 15)
# plt.ylabel('Success rate',fontsize = 15)
# ax.set_title("Success rates of the BHG strategy and the UHG strategy using the PSO", fontsize = 8)
# ax.set_title("Success rates of the BHG strategy and the UHG strategy using the ABC optimization", fontsize = 8)
# ax.set_title("Success rates of the BHG strategy and the UHG strategy using the APF", fontsize = 9)
plt.xlabel('Workspaces index',fontsize = 15)
plt.ylabel('Mean path length',fontsize = 15)
# ax.set_title("Mean path lengths of the BHG strategy and the UHG strategy using the PSO", fontsize = 8)
ax.set_title("Mean path lengths of the BHG strategy and the UHG strategy using the ABC optimization", fontsize = 8)
# ax.set_title("Mean path lengths of the BHG strategy and the UHG strategy using the APF", fontsize = 9)


plt.show()