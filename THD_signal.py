import numpy as np
fs = 10000    # Tan so lay mau
f1 = 100      # Tan so song
k = 30        # So bac song hai cao nhat

# Tao song sin
def sinwave():
      t = np.arange(0, 1, 1/fs)
      x = np.sin(2*np.pi*f1*t) 
      return x

# Tao song tam giac
def trianglewave():
      t = np.arange(0, 1, 1/fs)  
      period = 1 / f1                      # Chu ky
      t_wave = np.mod(t, period) / period  # Tinh khoang cach thoi gian giua cac diem song
      triangle_wave = np.abs(np.mod(t_wave + 0.25, 1) - 0.5) * 2 - 1  #Tao song
      return triangle_wave

# Tao song vuong
def squarewave():
      t = np.arange(0, 1, 1/fs)
      square_wave = np.where(np.sin(2*np.pi*f1*t) >= 0, 1, -1)      # Tao song vuong tu song sin: >=0 => gia tri 1, nguoc lai, nhan -1
      return square_wave

def THD(wave):
      spectrum = np.fft.fft(wave)   
      N = len(spectrum)
      P = np.abs(spectrum)**2/N 
      f = np.fft.fftfreq(N, 1/fs)

      # Tinh tong
      f1_idx = np.argmax(np.abs(f) == f1) # Tim vi tri tan so co ban
      P1 = P[f1_idx] #Cong suat cua tan so co ban
      P_harm = 0
      for i in range(2, k, 1):
            P_harm = P_harm +  P[f1_idx * i]  # Tong cong suat cua cac song hai


      # Tinh he so meo hai (THD)
      THD_signal = np.sqrt(P_harm) / np.sqrt(P1)
      return THD_signal

thd_sin = THD(sinwave())
thd_tri = THD(trianglewave())
thd_squa = THD(squarewave())

print('THD song sin = {:.2%}'.format(thd_sin))
print('THD song tam giac = {:.2%}'.format(thd_tri))
print('THD song vuong = {:.2%}'.format(thd_squa))








