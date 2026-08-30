# -*- coding: utf-8 -*-
"""每分鐘的分期準度 != 使用者看到的數字準度。

validate_staging.py 給的是逐分鐘的 kappa。但使用者讀的是「你昨晚深睡 62 分鐘」——
一晚一個數字。這支把 out-of-fold 預測按人彙總,算出那個數字的 MAE 與相關係數,
也就是真正該拿來做產品決策的量。2026-08-30 的結論就是這裡跑出來的:

    總睡眠時數  MAE 27.7 分 = 7% 誤差   r=0.91   <- 唯一撐得住的
    深睡分鐘    MAE 31.0 分 = 55% 誤差  r=0.15
    REM 分鐘    MAE 48.6 分 = 54% 誤差  r=0.42
    夜間清醒    MAE 27.7 分 = 59% 誤差  r=-0.11

⚠️ 資料集每人只有一晚(31 人 31 夜),所以 r 是「跨人」不是「跨夜」。
   「今晚比昨晚深睡多嗎」是同一個人跨夜的問題,這份資料完全回答不了 ——
   而那正是使用者真正會問的。跨人相關才 0.15 的東西,跨夜只會更差。

需要先跑 build_features.py 產生 features.npz。
"""
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.model_selection import GroupKFold
d=np.load("features.npz",allow_pickle=True)
X,y,g,shipped=d["X"],d["y"],d["g"],d["shipped"]
oof=np.zeros_like(y)
for tr,te in GroupKFold(n_splits=5).split(X,y,groups=g):
    assert not (set(g[tr])&set(g[te]))
    m=HistGradientBoostingClassifier(random_state=0).fit(X[tr],y[tr])
    oof[te]=m.predict(X[te])
W,L,D,R=0,1,2,3
def agg(pred):
    rows=[]
    for s in np.unique(g):
        k=g==s
        t,p=y[k],pred[k]
        rows.append(dict(
            n=k.sum(),
            t_deep=(t==D).sum(), p_deep=(p==D).sum(),
            t_rem=(t==R).sum(),  p_rem=(p==R).sum(),
            t_tst=(t!=W).sum(),  p_tst=(p!=W).sum(),
            t_waso=(t==W).sum(), p_waso=(p==W).sum()))
    return rows
def report(rows,label):
    print(f"\n=== {label} ===")
    print(f"{'量':<14}{'真值均值':>9}{'MAE':>8}{'相對誤差':>9}{'偏差':>8}{'跨人相關 r':>11}")
    for key,nm in (("tst","總睡眠時數"),("deep","深睡分鐘"),("rem","REM分鐘"),("waso","夜間清醒")):
        t=np.array([r["t_"+key] for r in rows],float); p=np.array([r["p_"+key] for r in rows],float)
        mae=np.abs(p-t).mean(); r=np.corrcoef(t,p)[0,1]
        print(f"{nm:<14}{t.mean():9.1f}{mae:8.1f}{100*mae/t.mean():8.0f}%{(p-t).mean():8.1f}{r:11.2f}")
    # 佔比版本
    print(f"{'--- 佔比版 ---':<14}")
    for key,nm in (("deep","深睡佔比"),("rem","REM佔比")):
        t=np.array([r["t_"+key]/max(r["t_tst"],1) for r in rows],float)
        p=np.array([r["p_"+key]/max(r["p_tst"],1) for r in rows],float)
        mae=np.abs(p-t).mean(); r=np.corrcoef(t,p)[0,1]
        print(f"{nm:<14}{100*t.mean():8.1f}%{100*mae:7.1f}pt{'':9}{100*(p-t).mean():7.1f}pt{r:11.2f}")
report(agg(oof),"最強模型(HistGradientBoosting, held-out)")
report(agg(shipped),"現行韌體")
print(f"\n注意:資料集每人只有一晚(31 人 31 夜),所以上面的 r 是「跨人」不是「跨夜」。")
print(f"     『今晚比昨晚深睡多』是同一個人跨夜的問題,這份資料完全回答不了。")
