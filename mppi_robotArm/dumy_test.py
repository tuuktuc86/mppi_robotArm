import numpy as np

sigma: np.ndarray = np.array([[20, 0.0], [0.0, 1]])
size_dim_u = 2
mu = np.zeros((size_dim_u))
#mu = np.array([5, 5])
#print(mu)

size_sample = 10000
size_time_step = 20
epsilon = np.random.multivariate_normal(mu, sigma, (size_sample, size_time_step))

#print(epsilon)
print(epsilon.shape)

print(f"1 mean = {np.mean(epsilon[:,0])}")
print(f"1 std = {np.std(epsilon[:,0])}")

print(f"1 max = {np.max(epsilon[:,0])}")
print(f"1 min = {np.min(epsilon[:,0])}")
print()
print(f"2 mean = {np.mean(epsilon[:,1])}")
print(f"2 std = {np.std(epsilon[:,1])}")
print(f"2 max = {np.max(epsilon[:,1])}")
print(f"2 min = {np.min(epsilon[:,1])}")

sample = np.random.normal(5, 0.7, 100)
print(np.min(sample))
print(np.max(sample))




