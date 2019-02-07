import pickle

f = open("calibResult", "rb")
amount = pickle.load(f)
print("amount", amount)
intrinsics = pickle.load(f)
for deviceId in range(amount):
    intrinsic = intrinsics[deviceId]
    print ("intrinsic:", intrinsic)

if amount == 2:
    R = pickle.load(f)
    T = pickle.load(f)
    print ("R", R)
    print ("T", T)
    
