'''
    This program converts a given integer to a base of your choice
'''

def converttobase(base, num):
    res,n = "", abs(num)
    while n:
        res = str(n%base) + res
        n = n//base
    return (((n < 0) * "-" )+ res) or "0"



print("enter base")
base = int(input())
print("enter number")
num = int(input())
res = converttobase(base, num)
print("The result is:"+ res)