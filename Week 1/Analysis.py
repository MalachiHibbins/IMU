import pandas as pd
import matplotlib.pyplot as plt

# read file 1
df1 = pd.read_csv("road1.csv", skiprows=1, na_values=1000)
df1 = df1.set_index("entry")
print(df1.head())

# read file 2
df2 = pd.read_csv("road2.csv", skiprows=1, na_values=1000)
df2 = df2.set_index("entry")
print(df2.head())

print("maximum speed on road 1: ", df1["speed detected/mph"].max())
print("minimum speed on road 2: ", df2["speed detected/mph"].min())
print("mean speed road 1: ", df1["speed detected/mph"].mean())
print("mean speed road 2: ", df2["speed detected/mph"].mean())

df1["speed detected/mph"].hist()
df2["speed detected/mph"].hist()
plt.plot()