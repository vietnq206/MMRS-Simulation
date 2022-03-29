import unittest


import MMRS_main as sim



class TestSum(unittest.TestCase):
    def test_list_int(self):

        rb = sim.robot(2*sim.sectorSize,2*sim.sectorSize,sim.robotRadius,(0,0,0))
        rb.pathAssign([[1,2,3,4],[1,2,3,4]])
        rb.indexPath = 1

        result1 = rb.reachNodePath()
        rb.x += 0.2
        rb.y += 0.2
        result2 = rb.reachNodePath()


        self.assertEqual(result1, True)
        self.assertEqual(result2, False)
    def test_get_angle(self):
        x = 1
        y = 1
        self.assertEqual( sim.get_angle(x,y),sim.np.pi/4)
        x = 1
        y = -1
        self.assertEqual( sim.get_angle(x,y),-sim.np.pi/4)
        x = -1
        self.assertEqual( sim.get_angle(x,y),-3*sim.np.pi/4)




if __name__ == '__main__':
    unittest.main()

