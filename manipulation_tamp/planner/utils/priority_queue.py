from heapq import *

class PriorityQueue:
    """ a generic priority queue
    """
    def __init__(self, items=None):
        """ a generic priority queue.

        Args:
            items: list of items, can be unsorted, must have __cmp__ implemented
        """
        if items is None:
            self.pq = []
        else:
            self.pq = items
        heapify(self.pq)
    
    def __contains__(self, item):
        return item in self.pq

    def __len__(self):
        return len(self.pq)

    def pop(self):
        return heappop(self.pq)

    def push(self, item):
        return heappush(self.pq, item)
    
    def pushpop(self, item):
        return heappushpop(self.pq, item)
    
    def replace(self, item):
        return heapreplace(self.pq, item)
    
    def heapify(self):
        heapify(self.pq)
    
    def peak(self):
        return self.pq[0]

def test_pq():
    pass

if __name__=="__main__":
    test_pq()