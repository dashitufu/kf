import numpy as np
import matplotlib.pyplot as plt

np.random.seed(987654321)   #������������Ա������
dtype = np.float64 

t = np.linspace(1,100,100) # ��1~100s�ڲ���100��
u = 0.6 # ���ٶ�ֵ���ȼ���ֱ���˶�ģ��
v0 = 5 # ��ʼ�ٶ�
s0 = 0 # ��ʼλ��
X_true = np.array([[s0], [v0]],dtype)
size = t.shape[0] + 1
dims = 2 # x, v, [λ��, �ٶ�]

Q = np.array([[1e1,0], [0,1e1]],dtype) # ����������Э�����������һ��������
R = np.array([[1e4,0], [0,1e4]],dtype) # �۲�������Э�������Ҳ��һ����������
# R_var = R.trace()
# ��ʼ��
X = np.array([[0], [0]],dtype) # ���Ƶĳ�ʼ״̬��[λ��, �ٶ�]����������Ҫ���Ƶ����ݣ�������v0��s0���룬Ҳ����Ĭ��Ϊ0�����Խ������ʱ��Խ��
P = np.array([[0.1, 0], [0, 0.1]],dtype) # �������Э�������ĳ�ʼֵ�����ݾ������
# ��֪�����Ա任����
F = np.array([[1, 1], [0, 1]],dtype) # ״̬ת�ƾ���
B = np.array([[1/2], [1]],dtype) # ���ƾ���
H = np.array([[1,0],[0,1]],dtype) # �۲����

# ��������ģ���Ƶ���������ʵλ��ֵ��ʵ�������в��������˼򵥵��˶�ģ�ͣ���ʵλ��Ҳ����֪����������ʹ����ֵ��Ŀ����ģ��۲��������ݺͲ�����������
# ����ʵ��Ӧ�õĿ������˲����ԣ�������Ҫ֪����ʵֵ������ͨ��Ԥ��ֵ�͹۲�ֵ����������Ź���ֵ���Ӷ����ϱƽ�����ֵ
real_positions = np.array([0] * size,dtype)

real_speeds = np.array([0] * size,dtype)
real_positions[0] = s0
# ʵ�ʹ۲�ֵ��ͨ������ֵ���Ϲ۲�����ģ���ã���ֵ�����۳�ʼ����Ϲ۲�����
measure_positions = np.array([0] * size,dtype)
measure_speeds = np.array([0] * size,dtype)
measure_positions[0] = real_positions[0] + np.random.normal(0, R[0][0]**0.5)
# ���Ź���ֵ��Ҳ���ǿ������˲��������ʵֵ�Ľ��Ʊƽ���ͬ���أ���ʼֵ�ɹ۲�ֵ����
optim_positions = np.array([0] * size,dtype)
optim_positions[0] = measure_positions[0]
optim_speeds = np.array([0] * size,dtype)

for i in range(1,t.shape[0]+1):
    #####################�ⲿ��������**************************
    # ��������ģ�ͻ�õ�ǰ���ٶȡ�λ����ʵֵ��ʵ��Ӧ���в���Ҫ����������ֻ��Ϊ��ģ�����ֵ�ͱȽ�
    w = np.array([[np.random.normal(0, Q[0][0]**0.5)], [np.random.normal(0, Q[1][1]**0.5)]],dtype=np.float32)
    #print("F\n",F,"\nX_true\n",X_true,"\nF*X_true\n",F @ X_true,"\nB*u\n",B*u,"\nw\n",w );
    X_true = F @ X_true + B * u + w
    #print(X_true)

    real_positions[i] = X_true[0][0]
    real_speeds[i] = X_true[1][0]
    #print(real_positions[i],real_speeds[i])

    v = np.array([[np.random.normal(0, R[0][0]**0.5)], [np.random.normal(0, R[1][1]**0.5)]],dtype=np.float32)
    #print(v[0][0],",",v[1][0],",")

    # �۲�������ڲ�����ʵ�Ĺ۲����ݣ�ע�����֮��Ĺ���
    Z = H @ X_true + v
    #Z[0]+=500
    #Z[1]+=10
    print(Z[0][0],Z[1][0])
    #####################�ⲿ��������**************************

    # �����ǿ������˲�����������
    X_ = F @ X + B * u
    #print("F\n",F,"\nB\n",B,"\nX\n",X,"\nX_\n",X_);

    P_ = F @ P @ F.T + Q
    #print(P_)

    # ע����������˳��
    K = P_@ H.T @ np.linalg.inv(H @ P_@ H.T + R)
    #print("H * P * H' + R\n",H @ P_@ H.T + R)
    #print("(H * P * H' + R)^-1\n", np.linalg.inv(H @ P_@ H.T + R))
    #print("K\n",K)
    X = X_ + K @ (Z - H @ X_)
    print(X[0][0],X[1][0])
    #print(Z)
    P = (np.eye(2) - K @ H ) @ P_
    #print(np.eye(2) - K @ H )

    # ��¼���
    optim_positions[i] = X[0][0]
    optim_speeds[i] = X[1][0]
    measure_positions[i] = Z[0][0]
    measure_speeds[i] = Z[1][0]
    
t = np.concatenate((np.array([0]), t))
plt.plot(t,real_positions,label='real positions')
plt.plot(t,measure_positions,label='measured positions')    
plt.plot(t,optim_positions,label='kalman filtered positions')

plt.legend()
plt.show()

plt.plot(t,real_speeds,label='real speeds')
plt.plot(t,measure_speeds,label='measured speeds')    
plt.plot(t,optim_speeds,label='kalman filtered speeds')

plt.legend()
plt.show()