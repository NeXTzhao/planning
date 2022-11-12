`RSS`**(Responsibility Sensitive Safety)**的责任敏感安全模型

规则1：**不要撞到前面的车（纵向距离）**

1. 同向行驶

   ![img](https://5b0988e595225.cdn.sohucs.com/images/20180613/ba966a26ea2d443599b569650d48efab.jpeg)

   ![image-20220426190142161](/home/next/.config/Typora/typora-user-images/image-20220426190142161.png)

   - 人类驾驶汽车和自动驾驶汽车的参数可以不同。比如自动驾驶汽车的反应时间一般会比人类短，而且自动驾驶汽车可以比人类驾车的刹车更有效。因此，自动驾驶汽车的αmin,brake可以设置得更大些。
   - 不同路况下可以设置不同的参数（湿滑路面、冰、雪等）

   

2. 反向行驶

   ![img](https://img-blog.csdn.net/20180825222558266?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

![image-20220426190043955](/home/next/.config/Typora/typora-user-images/image-20220426190043955.png)

**规则2.与侧方车保持一定安全距离（横向距离）**

因突然并线而导致的追尾，责任在实施并线的车辆。这在现阶段的交通法规中，也是这么定责的，但这并不意味着自动驾驶汽车就可以不作为，`RSS`模型认为，即使旁边车道的车辆突然插入了自动驾驶车辆的前方，只要有足够的距离进行刹车，自动驾驶汽车也必须进行刹车避免碰撞。

![img](https://pic2.zhimg.com/80/v2-61a561dfcc80b8d3bc114c78f8902815_720w.jpg)

![image-20220426190420679](/home/next/.config/Typora/typora-user-images/image-20220426190420679.png)

注：u为允许距离有一个大小波动的范围；这里的v和a都是侧向的速度和加速度

**规则3：路权的享有原则：不争抢路权**

![img](https://5b0988e595225.cdn.sohucs.com/images/20180613/605270ec5c9a4f319b91d48039f54d10.jpeg)

![img](https://5b0988e595225.cdn.sohucs.com/images/20180613/817e172f2db4444cac315d651e1222ec.jpeg)

`RSS`并不是刻板地以路权做为唯一判断，比如在上图图中，蓝车来不及刹车闯入了红车的车道，红车也要采取刹车以避免碰撞。(按规定蓝车应该停车，让红车先过)

![img](https://5b0988e595225.cdn.sohucs.com/images/20180613/5e5f3009383848fa871679d023b72252.jpeg)

甚至，`RSS`模型还可以支持轻微横向位移来避免撞击

**规则4：注意自动驾驶汽车周边的盲区**

下图中的车辆正在通过一排停车位，一名儿童突然以速度10km/h的速度跑过来（比如在追球）。根据计算，10km/h的速度必须要保持15m的安全距离才可能避免碰撞发生。但此时汽车侧方的视野只有0.3m，显然无法满足安全要求。在这种情况下，RSS模型做了如下定义：

-在车辆可以发现目标的第一时间（Te）到反应时间结束时（Te + ρ），车辆没有加速，且到发生撞击或者完全停下来的时刻（Ts），车辆一直以不低于αmin_brake的加速度在刹车。

-从Te到Ts这段时间内，车辆的平均速度低于行人的平均速度。

这种情况下车辆是没有责任的。这个定义隐含的论点是：在发生撞击的时刻，车辆的速度比行人的速度低，或者两者都移动得很慢，从而使撞击的伤害降到最低。

![img](https://5b0988e595225.cdn.sohucs.com/images/20180613/301268f945264876a8c2728a909c6e4e.jpeg)

在这些不确定性高，视眼有限的场景，基本上都是要考虑 worst case

![img](https://img-blog.csdn.net/2018072521452760?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



对于可靠性，使用了`NHTSA`的场景进行测试，cover了99.4%的碰撞情况。

![img](https://img-blog.csdn.net/20180725214606234?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

RSS描述的不是驾驶策略，而是对驾驶策略的结果进行安全判定，通过这种规则的设定，希来提前预防、及时避免事故的发生，以及在事故无法避免时，如何将伤亡降到最低。同时，保护OEM厂商的正当权益，避免受到来自社会或监管方不合理的责任分配。另一方面，RSS在定义不同场景下的危险情况、正确反应、事故责任划分中，是基于人类驾驶员的常识，而非仅仅法律法规。
