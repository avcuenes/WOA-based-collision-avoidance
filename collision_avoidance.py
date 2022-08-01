from mealpy.swarm_based.WOA import BaseWOA ,HI_WOA
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt


class CollisionAvoidance():
    def __init__(self,number_of_waypoint):
        self.number_of_wp = number_of_waypoint
        self.obstacle_x = [50,95,100]
        self.obstacle_y = [50,75,115]
        self.number_of_obs = len(self.obstacle_x)
        self.distance_between_wp = 2
        self.distance_from_obstacle = 20
        self.coeficient = 100
    
    def Euclidean_distance(self,point_one, point_two):
        return np.sqrt(pow(point_one[0]-point_two[0],2) + pow(point_one[1]-point_two[1],2))
        
    def constr_one(self,X,Y):
        """_summary_
        This function is constraint function to use equal distance

        Args:
            x (list): _description_ waypoint list x and y
        """
        
        lenpx = np.zeros(X.shape[0])
        count = 0
        
        
        for i in range(0,self.number_of_obs):
            x_center = self.obstacle_x[i]
            y_center = self.obstacle_y[i]
            
            d = np.sqrt((X - x_center) ** 2 + (Y - y_center) ** 2)
            
            inside = self.distance_from_obstacle > d
            
            cost = np.where(inside, self.coeficient /d, 0)
            
            if (inside.any()):
                count += 1
            
            lenpx += np.nanmean(cost)
        
        return lenpx
            
            
        
        return gx
                    
    def objective_function(self,x):
        fx = 0
        
        x[0] = 0                        # initial X position
        x[self.number_of_wp-1] = 150    # final X position
        
        x[self.number_of_wp] = 0        # initial Y position
        x[2*self.number_of_wp-1] = 150  # final Y position
        
        interpolator ='quadratic'
        # interpolator = 'cubic'
        # interpolator = 'slinear'
        # interpolator = 'zero'
        
        
        d = 50
        ns = 1 + (self.number_of_wp + 1) * d         # Number of points along the spline

        
        x1 = x[0:self.number_of_wp]
        y1 = x[self.number_of_wp:2*self.number_of_wp]
        
        
        t = np.linspace(0, 1, self.number_of_wp)
        
        CSx = interp1d(t, x1 ,kind=interpolator, assume_sorted=True)
        CSy = interp1d(t, y1, kind=interpolator, assume_sorted=True)
        
        # Coordinates of the discretized path
        s = np.linspace(0, 1, ns)
        
        Px = CSx(s)
        Py = CSy(s)
       
        dX = np.diff(Px)
        dY = np.diff(Py)
        L = np.sqrt(dX ** 2 + dY ** 2)
        
        gx = self.constr_one(Px,Py)
       
       
        Cost = L + (1+gx[0:len(L)])
       
        return Cost
    
    def run(self):
        lb = []
        ub = []
        for i in range(0,self.number_of_wp*2):
            lb.append(0)
            ub.append(150)
            
        """lb = np.zeros(self.number_of_wp*2)
        ub = np.ones(self.number_of_wp*2)*150"""
        print(lb,len(lb))
        print(ub,len(ub))
        problem_dict1 = {
        "fit_func": self.objective_function,
        "lb": lb,
        "ub": ub,
        "minmax": "min",
        }
        
        
        
        ## Run the algorithm
        model1 = BaseWOA(problem_dict1, epoch=80  , pop_size=200)
        best_position, best_fitness = model1.solve()
        print(f"Best solution: {best_position}, Best fitness: {best_fitness}")
        self.plot(best_position)
        
        
    def plot(self,solutions):
        
        x = solutions[0:self.number_of_wp]
        y = solutions[self.number_of_wp:2*self.number_of_wp]
        
        
                
        print(x)
        print(y)         
        plt.scatter(x, y)
        plt.scatter(self.obstacle_x,self.obstacle_y,s=( self.distance_from_obstacle)**2 )
        plt.show()
        
        


if __name__=="__main__":
    A = CollisionAvoidance(5)
    A.run()