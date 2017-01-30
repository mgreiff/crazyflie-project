"""This module contains all functions concerning the trajectories and motion planning
of the crzyflie."""

__license__ = "MIT"
__docformat__ = 'reStructuredText'

import numpy as np
import sys, os, json
import itertools
from scipy.linalg import inv, cholesky
from scipy.signal import cont2discrete
import scipy.io
from math import sin, cos, sqrt, ceil
from copy import copy
from mpl_toolkits.mplot3d import Axes3D # Obly used for 3D plotting
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines

class Trajectory(object):
    """The Trajecory class is used to host, manipulate, communicate and
    evaluate trajectories for evaluation in the Crazyflie UAV. 

    The trajectories are represented in terms of individual splines, each
    stored as a spline object which may be converted to a packet and sent
    to the UAV. The trajectory 
    """

    def __init__(self, directory, filename, fileformat):
        """Creates a trajectory object from a file of a specific format from a
        trajectory directory.
        """
        self.isComplete = False         # Flag set true ifthe assertions pass
        self.isConsistent = False       # Flag set true ifthe assertions pass
        self.isFeasible = False         # Flag set true ifthe assertions pass
        self.set_directory(directory)
        self.nDim = 5                   # The number of flat output dimensions
                                        # (0-3) and event timeline (4)
        self.load(filename, fileformat)
        self.timeStep = 0.02            # Evaluate at 50 Hz as in the firmware


    def load(self, filename, fileformat):
        """Loads a trajectory for use in outer control and visualization

        :param filename: Name of the file to load
        :param fileformat: Format of the file to load (json, csv or mat)
        :type filename: string
        :type fileformat: string
        """
        path = os.path.join(self.directory, "%s.%s" %(filename, fileformat))
        
        ### Loads trajectory data ###
        if fileformat == 'json':
            try:
                with open(path) as trajectoryFile:
                    data = json.load(trajectoryFile)
                    packets = data['packets']
                    settings = data['settings']
            except:
                raise Exception('Could not load the .json trajectory "%s"' %(filename))
        elif fileformat == 'mat':
            raise Exception('The mat format is not yet supported')
        elif fileformat == 'csv':
            raise Exception('The csv format is not yet supported')
        else:
            raise Exception('The format %s is not yet supported' % (fileformat))

        ### Erases the old object and creates a new one ###
        self.trajectory = self.nDim * [None]
        self.info = settings['info']
        self.circular = settings['circular']
        
        # Old files will have four dimensions (no events) 
        self.number = settings['number']
        if len(self.number) == 4:
            self.number = self.number + [0]

        # Create spline objects
        for ii in range(self.nDim):
            self.trajectory[ii] = self.number[ii] * [None]
        for packet in packets:
            dim = packet['dimension']
            ind = packet['index']
            identifier = packet['type']
            time = packet['time']
            data = packet['data']
            self.trajectory[dim][ind] = Spline(time,
                                               data,
                                               identifier,
                                               ind,
                                               dim,
                                               trajectory=self)

        ### Checks that no dimension is empty and sets is safely to zero    ###
        ### indicating zero valued step in the spatial dimensions or poewr  ###
        ### in the event time line                                          ###
        for dim in range(self.nDim):
            if not self.trajectory[dim]:
                self.issue_warning('Found empty dimension %i in the trajectory.' % dim)
                self.number[dim] = 1
                self.trajectory[dim]=[Spline(time = 10.0,
                                             data = [0.0,0.0,0.0,0.0,0.0,0.0],
                                             identifier = 0,
                                             index = 0,
                                             dimension = dim,
                                             trajectory=self)]
                                               
        ### Checks that the trajectory is fully populated, i.e. that the    ###
        ### number of indices correspond to the number of entries along     ###
        ### each flat output dimension
        self.assert_complete()

        ### Precompute the array times                                      ###
        self.times = self.nDim*[None]
        for ii in range(self.nDim):
            time = [spline.time for spline in self.trajectory[ii]]
            self.times[ii] = np.cumsum(time)
       
        ### Check trajectory consistency and warn if inconsistent           ###
        self.assert_consistency()

        ### Initialize graphics
        # TODO this is currently only supported for the spatial trajectories 
        # and should be made available for the events as well
        for ii in range(4):
            for spline in self.trajectory[ii]:
                spline.initialize_graphics()
       
    def save(self, filename):
        """Saves the current trajectory to a JSON and writes it to filename
        in the previously defined trajectory file directory.
        """
        ### Input check ###
        if type(filename) is not str:
            raise ValueError('The filename must be of type str')
        
        ### Writes data and saves trajectory ###
        packets = []
        number = []
        for dim in range(4):
            num = len(self.trajectory[dim])
            number += [num]
            for ind in range(num):
                spline = self.trajectory[dim][ind]
                packets.append(spline.packet(num))
        settings = {'info':self.info, 'circular':self.circular, 'number':number}
        trajData = {'packets':packets, 'settings':settings}

        path = os.path.join(self.directory, filename)
        with open(path, 'w') as outfile:
            json.dump(trajData, outfile , separators=(',', ':'), sort_keys=True, indent=4)

    def evaluate_trajectory(self, time):
        """Evaluates the trajectory at a specific point in time, and lets the
        reference remain constant if the desired time does not exist in the
        interval on which the trajectory is defined.

        :param time: The time at which the trajectory is to be evaluated
        :type time: float
        :returns: x, y, z, yaw
        :rtype: np.array, np.array, np.array, np.array
        """
        
        values = np.zeros((4,5))
        
        # Iterate over the flat output dimensions
        startTime = 0.0
        for ii in range(4):
            index = int(sum(self.times[ii] < time))
            if index < 1:
                index = 0
                startTime = 0.0
            else:
                if index == len(self.times[ii]):
                    index = len(self.times[ii]) - 1
                startTime = self.times[ii][index-1]
                
            spline = self.trajectory[ii][index]
            values[ii,:] = spline.evaluate(time - startTime)

        x = values[0,:]
        y = values[1,:]
        z = values[2,:]
        yaw = values[3,:]
        return x, y, z, yaw
        
    def set_directory(self, directory):
        """
        Sets the directory of the trajectory object
        
        :param directory: Directory to which files are stored and loaded
        :type directory: string
        """
        # Todo check that the directory exists
        self.directory = directory

    def assert_complete(self):
        """
        Evaluates the consistency of all trajectories in terms of each flat
        output dimension containing non None entries
        """
        self.isComplete = True
        for ii in range(4):
            if None in self.trajectory[ii]:
                ind = self.trajectory[ii].index(None)
                self.issue_warning('Missing index %i in dimension %i' % (ii, ind))
                self.isComplete = False

    def assert_consistency(self):
        """
        Evaluates the consistency of all trajectories in terms of continuity
        in position and first derivative.
        """
        self.isConsistent = True
        for ii in range(4):
            if None in self.trajectory[ii]:
                ind = self.trajectory[ii].index(None)
                self.issue_warning('Missing index %i in dimension %i' % (ii, ind))
                self.isComplete = False

    def assert_feasibility(self):
        """
        Evaluates the dynamical feasibility of all trajectories
        """
        raise Exception('Not yet written')

    def issue_warning(self, message):
        """
        Issues a warning to the user
        """
        print(message)

    def visualize_interactive(self, dimension):
        """
        Visualizes trajectory as it is currently set, using the pre-evaluated
        splines to enable interaction with the plots. This effectively allows
        the solving of the polynomial spline QP-problem and setting of
        bezier curves interatively.
        """
        dimensions = ['x','y','z','yaw']
        if not dimension in dimensions:
            raise Exception(('The entered dimension %s%s is not a valid'+
                             'dimension' % str(dimension)))
        dim = dimensions.index(dimension)

        ymin, ymax = 1.0,5.0
        xl=[0.0, self.times[dim][-1]]
        yl=[ymin, ymax]
        fig, ax = plt.subplots()
        ax.set(xlim = xl, ylim = yl)
        
        graphicsObjects = []
        for spline in self.trajectory[dim]:
            for obj in spline.graphicsObjects:
                ax.add_artist(obj)
                graphicsObjects.append(obj)
        go = GraphicsObject(graphicsObjects)
        plt.show()

    def visualize_fixed(self, dimension, projection):
        """
        Visualizes trajectory as it is currently set, evaluating the complete
        trajectory across all dimensions based on the dimensions specified by
        the user. The evaluation of flatness is costly and requires synchronous
        evaluation across all dimensions, hence the distinguishing between
        interactive and fixed visualization.
        """
        
        # Input check
        dimension = list(set(dimension))        

        # Evaluation of trajectory        
        timeMax = max([ct[-1] for ct in self.times])
        N = int(ceil(timeMax / self.timeStep))
        time = np.zeros(N)
        data = {'x':np.zeros((N,5)), 'y':np.zeros((N,5)),
                'z':np.zeros((N,5)), 'yaw':np.zeros((N,5))}
        for ii in range(N):
            tt = ii*self.timeStep
            data['x'][ii,:], data['y'][ii,:], data['z'][ii,:], data['yaw'][ii,:] = self.evaluate_trajectory(tt)
            time[ii] = tt

        numFlatDim = len(dimension)
        if numFlatDim == 0:
            raise Exception('A flat output dimension must be specified')

        if '1D' in projection:
            # One dimensional plots
            if numFlatDim > 0:
                maxDerivOrder = 3
                labels = ['position', 'velocity', 'acceleration', 'jerk', 'snap']
                fig = plt.figure(1)
                for ii in range(0,maxDerivOrder):
                    plt.subplot(maxDerivOrder,1,ii+1)      
                    yStr = ''
                    for dim in dimension:
                        plt.plot(time, data[dim][:,ii])
                        yStr += '%s,' % dim
                    plt.xlabel('Time')
                    plt.ylabel(labels[ii])
                fig.tight_layout()

        if '2D' in projection:
            # Two dimensional zeroth order plots
            combinations = [pair for pair in itertools.combinations(dimension, 2)]
            if numFlatDim > 1:
                fig = plt.figure(2)
                numCombinations = len(combinations)
                for ii in range(numCombinations):
                    plt.subplot(numCombinations,1,ii+1)
                    pair = combinations[ii]
                    plt.plot(data[pair[0]][:,0], data[pair[1]][:,0])
                    plt.xlabel(pair[0])
                    plt.ylabel(pair[1])
                fig.tight_layout()

        # Three dimensional trajectory 
        if '3D' in projection:  
            if 'x' in dimension and 'y' in dimension and 'z' in dimension:
                fig = plt.figure(3)
                ax = fig.gca(projection='3d')
                ax.plot(data['x'][:,0], data['y'][:,0], data['z'][:,0])
                ax.set_xlabel('x(t)')
                ax.set_ylabel('y(t)')
                ax.set_zlabel('z(t)')

        plt.show()

class Spline(object):
    """The Spline class is used to manipulate, communicate and evaluate splines
    before transmission to the Crazyflie UAV. The transmission can be done by
    real-time evaluation or preloading of entire trajectories.
    
    There are two different kinds of trajectories which may be used to creating 
    applications, the dynamical splines in the flat output space, and the event
    based splines defining when to for instance start, land and capture frames.
    
    **Dynamical splines**
    The dynamical splines specify movement in flat output space with four
    common methods of parametrizing trajectories, each referred to by an
    integer identifier. In order to use the property of differential flatness
    the trajectories need to specify non-singular derivtives up tothe fourth
    order in x, y, and z (translation) and the second derivative of yaw
    (rotation) about the body z-axis. The data field specifies a trajectory
    which is to be followed during a time specified by the time field.
    
    * 0 - Linear trajectory, data = [p0,. ,. ,. ,. ,. ] with x(t) = p0*\theta(t)
    
    * 1 - Polynomial trajectory, data = [p0,p1,p2,p3,p4,p5] with x(t) = \sum\limits_{i=0}^5 p_i*t^i
    
    * 2 - Sinusoid trajectory, data = [A ,B ,w ,p ,. ,. ] with x(t) = \sum\limits_{i=0}^5 p_i*t^i
    
    * 3 - Bezier trajectory, data = [b0,b1,b2,b3,. ,. ] with x(t) = \sum\limits_{i=0}^5 p_i*t^i
    
    **Event splines**
    The event splines are also referred to with an integer identifier. The data
    field specifies a trajectory which is to be followed during a time
    specified by the time field.
    
    * 4 - Event trajectory, data = [s1,s2,s3,s4,s5,s6]
    
    """
    def __init__(self, time, data, identifier, index, dimension, trajectory):
        self.time = time
        self.data = data
        self.identifier = identifier
        self.dimension = dimension
        self.index = index
        self.trajectory = trajectory

        self.valueArray = None       # An array valued array
        self.posArray = None         # An array
        self.timeArray = None        # A float valued array
        self.N = 20                  # Number of points in which the trajectory
                                     # is represented
        self.represent()
    
    def packet(self, number):
        """
        Generates a packet from the spline which may be used for storing the
        spline object or sending the data to the Crazyflie UAV
        
        :param dimension: The dimension of the spline (x=0, y=1, z=2, yaw=3, event=4)
        :param index: Index of the spline in the complete trajectory along the specified dimension
        :param number: The total number of splines in the specified dimension
        :type dimension: int
        :type index: int
        :type number: int
        :returns: packet
        :rtype: dictionary
        
        """
        return {'data':self.data,
                'dimension':self.dimension,
                'number':number,
                'index':self.index,
                'time':self.time,
                'type':self.identifier
                }

    ###########################################################################
    ### Methods for computationally efficient evaluation of splines         ###
    ###########################################################################
    def represent(self):
        """
        Represents the entire spline internally using the evaluate function
        for plotting purposes
        """
        self.timeArray = np.linspace(0, self.time, self.N)
        self.valueArray = np.zeros((self.N,5))
        self.flatnessArray = self.N * [None]
        for ii in range(self.N):
            t = self.timeArray[ii]
            self.valueArray[ii] = self.evaluate(t)
        self.posArray = self.valueArray[:,0]
        
    def evaluate(self, time):
        """
        Evaluates the spline at a specified time
        """
        if time < 0:
            time = 0
        elif time > self.time:
            time = self.time
        if self.identifier == 0:
            return self._eval_steps(time)
        elif self.identifier == 1:
            return self._eval_polynomial(time)
        elif self.identifier == 2:
            return self._eval_sinusoid(time)
        elif self.identifier == 3:
            return self._eval_bezier(time)
        

    def _eval_sinusoid(self, time):
        """Evaluates a sinusoid trajectory

        :param time: The time at which the trajectory is to be evaluated
        :type time: float
        :returns: flatPoint
        :rtype: np.array

        """
        A = self.data[0]
        B = self.data[1]
        w = self.data[2]
        p = self.data[3]
        
        vSin = sin(w * time + p)
        vCos = cos(w * time + p)
        w2 = w * w
        w3 = w2 * w
        w4 = w3 * w
        flatPoint = np.array([A * vSin + B,
                              A * w * vCos,
                              -A * w2 * vSin,
                              -A * w3 * vCos,
                              A * w4 * vSin])
        return flatPoint

    def _eval_polynomial(self, time):
        """Evaluates a polynomial trajectory

        TODO: longer explanation

        :param time: The time at which the trajectory is to be evaluated
        :type time: float
        :returns: flatPoint
        :rtype: np.array

        """
        t0 = 1.0
        t1 = time
        t2 = t1 * t1
        t3 = t2 * t1
        t4 = t3 * t1
        t5 = t4 * t1
        t = [t0, t1, t2, t3, t4, t5]
        c = copy(self.data)
        flatPoint = np.array([0.0,0.0,0.0,0.0,0.0]) 
        maxorder = 5
        # Iterates over the flat output derivatives
        for order in range(maxorder):
            # Write values
            for n in range(maxorder - order):
                flatPoint[order] = flatPoint[order] + t[n] * c[n+order]
            # Derive polynomial coefficients
            count = 0.0
            for ii in range(order, maxorder):
                c[ii] = count * c[ii]
                count = count + 1.0
        return flatPoint

    def _eval_steps(self, time):
        """Evaluates a step trajectory

        :param time: The time at which the trajectory is to be evaluated
        :type time: float
        :returns: flatPoint
        :rtype: np.array

        """
        # TODO: include filtering
        p = self.data[0]
        flatPoint = np.array([p,0,0,0,0])
        return flatPoint

    def _eval_bezier(self, time, spline):
        """Evaluates a cubic bezier trajectory

        :param time: The time at which the trajectory is to be evaluated
        :type time: float
        :returns: flatPoint
        :rtype: np.array

        """
        flatPoint = np.array([0,0,0,0,0])
        return flatPoint

    ###########################################################################
    ### Methods to enable interactive graphics                              ###
    ###########################################################################

    def initialize_graphics(self):

        # General settings
        self.ellipseRadius = 0.05

        # Positional and time offsets
        if self.index == 0:
            self.timeOffset = 0.0;
        else:
            self.timeOffset = self.trajectory.times[self.dimension][self.index - 1]
        self.posOffset = 0.0;
        
        ### Objects used in all applications ###

        # Start point
        self.sPoint = patches.Ellipse((self.timeArray[0], self.posArray[0]), width=0.0, height=0.0, fc='r', alpha=0.9, visible=False)
        self.sPoint.parent = self
        self.sPoint.identifier = 'Start'
        
        # Finish point
        self.fPoint = patches.Ellipse((self.timeArray[-1], self.posArray[-1]), width=0.0, height=0.0, fc='g',alpha=0.9, visible=False)
        self.fPoint.parent = self
        self.fPoint.identifier = 'Finish'
        
        # Spline line
        self.sLine = lines.Line2D(self.timeArray + self.timeOffset,
                                  self.posArray + self.posOffset)
        self.sLine.parent = self
        self.sLine.identifier = 'Line'
        
        self.graphicsObjects = [self.sLine, self.fPoint, self.sPoint]

    def update_graphics(self):
        """
        Update graphics based on the current data in the object
        """
        # Point representation height an width relative to the axis limits
        xLimits = plt.gca().get_xlim()
        yLimits = plt.gca().get_ylim()
        width = self.ellipseRadius * (xLimits[1] - xLimits[0])
        height = self.ellipseRadius * (yLimits[1] - yLimits[0])
        
        # General data settings
        self.sPoint.center = (self.timeArray[0] + self.timeOffset,
                             self.posArray[0] + self.posOffset)
        self.sPoint.width, self.sPoint.height = width, height
        self.fPoint.center = (self.timeArray[-1] + self.timeOffset,  
                             self.posArray[-1] + self.posOffset)
        self.fPoint.width, self.fPoint.height = width, height
        self.sLine.set_xdata(self.timeArray + self.timeOffset)
        self.sLine.set_ydata(self.posArray + self.posOffset)

    def _re_select(self):
        """
        Settings for when the spline is selected
        """
        self.sPoint.set_visible(True)
        self.fPoint.set_visible(True)
        self.sLine.set_linewidth(3)

    def _un_select(self):
        """
        Settings for when the spline is not selected
        """
        self.sPoint.set_visible(False)
        self.fPoint.set_visible(False)
        self.sLine.set_linewidth(1)

class GraphicsObject(object):
    """
    The graphics object is used to enable interactive plotting in the
    visualization and generation of trajectories.
    """

    def __init__(self, artists, tolerance=5):
        """
        Creates a Graphics object which hosts
        """
        for artist in artists:
            artist.set_picker(tolerance)
        self.artists = artists
        self.currently_dragging = False
        self.current_artist = None
        self.previous_artist = None
        self.offset = (0, 0)

        for canvas in set(artist.figure.canvas for artist in self.artists):
            canvas.mpl_connect('button_press_event', self.on_press)
            canvas.mpl_connect('button_release_event', self.on_release)
            canvas.mpl_connect('pick_event', self.on_pick)
            canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_press(self, event):
        """
        Callback for a mouse click event in the interactive mode
        """
        self.currently_dragging = True
        

    def on_release(self, event):
        """
        Callback for a release event in the interactive mode
        """
        self.currently_dragging = False
        self.previous_artist = self.current_artist
        self.current_artist = None

    def on_pick(self, event):
        """
        Callback for a pick event in the interactive mode, when picking and
        artist.
        """
        if self.current_artist is None:
            if self.previous_artist is not None:
                self.previous_artist.parent._un_select()
            self.current_artist = event.artist
            self.current_artist.parent._re_select()
            
            x1, y1 = event.mouseevent.xdata, event.mouseevent.ydata
            if (self.current_artist.identifier == 'Start' or
                self.current_artist.identifier == 'Finish'):
                x0, y0 = event.artist.center
            elif self.current_artist.identifier == 'Line':
                xData, yData = event.artist.get_data()
                x0, y0 = xData[0], yData[0]
            self.offset = (x0 - x1), (y0 - y1)
    
    def on_motion(self, event):
        """
        Callback for a motion event in the interactive mode, when moving an
        artist.
        """
        if not self.currently_dragging:
            return
        if self.current_artist is None:
            return
        
        spline = self.current_artist.parent
        if self.current_artist.identifier == 'Start':
            spline.posOffset = event.ydata + self.offset[1] - spline.posArray[0]
        if self.current_artist.identifier == 'Finish':
            spline.posOffset = event.ydata + self.offset[1] - spline.posArray[-1]
        elif self.current_artist.identifier == 'Line':
            spline.timeOffset = event.xdata + self.offset[0] - spline.timeArray[0]
            spline.posOffset = event.ydata + self.offset[1] - spline.posArray[1]

        spline.update_graphics()
        self.current_artist.figure.canvas.draw()

if __name__ == "__main__":
    import doctest
    doctest.testmod()
    if 1:
        d = '/Users/mgreiff/Desktop/crazyflie-trajectory/trajectories'
        fn = 'p_1'
        ff = 'json'
        t = Trajectory(d, fn, ff)
        t.visualize_interactive('x')
        t.visualize_fixed(['x','y','z'], ['1D', '2D', '3D'])
        #t.save('t_1_event.json')
