'''
    Basic Binary Search algo template
    Given an sorted array of integers(ascending), find an integer.
    If present return its index else return -1
'''

def binarysearch(nums, target):

    start,last = 0, len(nums-1)
    mid = (start + last)/2

    while(start <= last):

        if(nums[mid] == target):
            return mid
        
        elif(nums[mid] > target):
            start = mid + 1

        else:
            last = mid - 1
        
        mid = (start + last)/2

    return -1

