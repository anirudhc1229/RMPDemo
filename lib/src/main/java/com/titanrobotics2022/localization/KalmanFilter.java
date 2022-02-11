package com.titanrobotics2022.localization;

import org.ejml.data.DMatrix2;
import org.ejml.data.DMatrix2x2;
import static org.ejml.dense.fixed.CommonOps_DDF2.*;

public class KalmanFilter {
    private final DMatrix2[] zs;
    private final DMatrix2x2[] precs;
    private final DMatrix2[] means;
    private final DMatrix2x2[] covs;
    private final DMatrix2x2 drift;
    private final DMatrix2 v = new DMatrix2();
    private final DMatrix2x2 m = new DMatrix2x2();
    private int inconsistent;

    public KalmanFilter(int order, DMatrix2x2 drift) {
        if(order < 0)
            throw new IllegalArgumentException("Order must be nonnegative.");
        this.drift = drift.copy();
        zs = new DMatrix2[order+1];
        precs = new DMatrix2x2[order+1];
        means = new DMatrix2[order+1];
        covs = new DMatrix2x2[order+1];
        for(int i=0; i<=order; i++){
            zs[i] = new DMatrix2();
            precs[i] = new DMatrix2x2();
            means[i] = new DMatrix2();
            covs[i] = new DMatrix2x2();
        }
        inconsistent = (1<<(order+1))-1;
    }

    public void update(int order, DMatrix2 pred, DMatrix2x2 prec) {
        mult(prec, pred, v);
        addEquals(zs[order], v);
        addEquals(precs[order], prec);
        inconsistent |= 1<<order;
    }

    public void step(double time) {
        double alpha;
        for(int i=0; i<zs.length; i++){
            invert(precs[i], covs[i]);
            mult(covs[i], zs[i], means[i]);
        }
        for(int i=1; i<zs.length; i++){
            alpha = 1;
            for(int j=i+1; j<zs.length; j++){
                alpha *= time / i;
                scale(alpha, means[j], v);
                scale(alpha, covs[j], m);
                addEquals(means[i], means[j]);
                addEquals(covs[i], covs[j]);
            }
            alpha *= time / zs.length;
            scale(alpha, drift, m);
            addEquals(covs[i], m);
        }
        for(int i=0; i<zs.length; i++){
            invert(covs[i], precs[i]);
            mult(precs[i], means[i], zs[i]);
        }
        inconsistent = 0;
    }

    public void getPred(int order, DMatrix2 out) {
        if(((inconsistent >> order) & 1) == 1){
            invert(precs[order], covs[order]);
            mult(covs[order], zs[order], means[order]);
            inconsistent ^= 1<<order;
        }
        scale(1, means[order], out);
    }
    public DMatrix2 getPred(int order) {
        DMatrix2 res = new DMatrix2();
        getPred(order, res);
        return res;
    }

    public void getCov(int order, DMatrix2x2 out) {
        if(((inconsistent >> order) & 1) == 1){
            invert(precs[order], covs[order]);
            mult(covs[order], zs[order], means[order]);
            inconsistent ^= 1<<order;
        }
        scale(1, covs[order], out);
    }
    public DMatrix2x2 getCov(int order) {
        DMatrix2x2 res = new DMatrix2x2();
        getCov(order, res);
        return res;
    }
}
