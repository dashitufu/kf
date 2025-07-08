# kf
Kalman filter 卡尔曼滤波器

手搓版，无第三方

1，Kf_Test_1()

	经典卡尔曼滤波器，只支持线性模型。与参考代码kf_2d.py的运算结果完全一致
  
	状态只有两个p,v，  观测也是p,y
  
	仅为调通数学过程用，不含任何营养，没有任何优化甚至有点慢

2，Kf_Test_2()

	参考程序并没有沿路对Q, R两个协方差进行更新，感觉反应不了局部样本形态

	加一格大小为20样本的滑动窗口，想当然以为更好反映局部形态

 	事实上没用，反而恶化了最优结果。问题出在Q的更新上。并没有很好的方法对Q进行滑动窗口更新

  	思路可以只优化R

3，Kf_Test_3()

	虽然Kf_Test_2()加窗未遂，但是协方差窗口感觉是常用操作，每次求期望方差会非常慢

 	对窗口的移动进行算法优化，采用一加一减法，窗口操作速度感人

  	将在附录给出协方差窗口的一加一减算法详细描述



附录
——协方差滑动窗口的一加一减法

设滑动窗口的样本数为n，第k次的各种变量为

	xi	i = k, k+1, ..., k+n	样本
	∑xi						样本值之和
	∑(xi - E)					样本值减期望之和，非一阶中心矩
	∑(xi - E)² 					非方差
	∑(xpj - Ep)(xqj - Eq)
	E(x)						期望
	D(x)						方差
	Cov(xp,xq)					协方差

第k+1次的各种变量为

	xj'	j = k+1,k+2, ..., k+n	样本
	∑'xj						样本值之和
	∑'(xj - E')					样本值减期望之和，非一阶中心矩
	∑'(xj - E')² 					非方差
	∑'(xpj - E')(xqj - E')				非协方差
	E'(x)						期望
	D'(x)						方差
	Cov(xp, xq)					协方差

X = xk, xk_1,  ..., xk_n_1

∑xi = xk + xk_1 +  ... + xk_n_1

...

那么，第 k+1次的更新为

1, ∑'xj	=  	xk_2 +  ... + xk_n_1 + xk_n
	=	∑xi - xk + xk_n
 
2, E'(X) 	=	1/n * ∑'xj

3, ∑'(xi - E') =	0

4, ∑'(xj - E')²

	设 Δe = E - E' , 
	则 xj - E' 	= xj - E' + E - E 
			= (xj - E) +  Δe
	(xj - E')² = [ (xj - E) +  Δe]² =	(xj - E)² + 2*Δe*(xj - E) + Δe²	
	故此，原式 =∑' (xj - E)² + 2*Δe*∑'(xj - E) + n*Δe²	此式可以看成三部分

   展开第一部分
   
	∑'(xj - E)² = (xk_2 - E)² + (xk_3 - E)² + ... + (xk_n_1 - E)² + (xk_n - E)²
			= ∑(xi- E)²  - (xk - E)² + (xk_n - E)²		取得一加一减形式

   展开第二部分
   
	∑'(xj - E) = (xk_2 - E) + (xk_3 - E) + ... + (xk_n - E)
			=   ∑(xk - E) - (xk-E) + (xk_n -E)
			=   (xk_n -E)  - (xk-E) 					取得一加一减形式

  故此，全部合在一起
  
	= ∑(xi- E)²  - (xk - E)² + (xk_n - E)²  + 2*Δe* [(xk_n -E)- (xk-E) ]  + n*Δe²

5，∑'(xpj - E')(xqj - E')
	
	设Δe = E - E' 
	则	 xpj - Ep' =  (xpj - Ep) +  Δep
		 xqj - Eq' =  (xqj - Ep) +  Δeq
	(xpj - Ep')* (xqj - Eq') = [(xpj - Ep) +  Δep] * [(xqj - Eq) +  Δeq]
		= (xpj - Ep) *  (xqj - Eq) + Δeq* (xpj - Ep)  + Δep*(xqj - Eq)] +  Δep*Δeq
	
	∑'(xpj - E')(xqj - E') = 	∑'(xpj - Ep) *  (xqj - Eq) + 					第一部分
						Δeq*∑' (xpj - Ep)  +  					第二部分
						Δep*∑' (xqj - Eq) + 					第三部分
						n*Δep*Δeq 						看成4部分
	
	第一部分 = (xk2p - E)*(xk2q -E) + (xk3p - E)*(xk3q -E) + ... + (xkn_1p - E)*(xkn_1q -E) + (xknp - E)*(xknq -E)
			= cov(xp,xq) - (xk1p - E)*(xk1q - E) + (xknp - E)*(xknq -E)		取得一加一减形式

	第二，三部分
	Δeq*∑' (xpj - Ep)  + Δep*∑' (xqj - Eq)  + n*Δep*Δeq
		= Δeq*∑'xpj + Δep*∑'xqj -  n*Δeq*Ep - n*Δep*Eq + n*Δep*Δeq
		= Δeq*∑'xpj + Δep*∑'xqj - n*(Δeq*Ep + Δep*Eq + Δep*Δeq)

	第四部分也OK
	
	总结：
	∑'(xpj - Ep')(xqj - Eq')  = cov(xp,xq) - (xk1p - E)*(xk1q - E) + (xknp - E)*(xknq -E) + 
						Δeq*∑' (xpj - Ep) +
						Δep*∑' (xqj - Eq) +
						n*Δep*Δeq

6，D'(x) = 1/n * ∑'(xj - E')²

7, Cov'(x) = 1/n * ∑'(xpj - E')(xqj - E') 

