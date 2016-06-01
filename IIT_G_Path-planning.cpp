//P.KARTHIKEYA SHARMA 
//IIT-GUWAHATI
//Motion planning in IIT-GUWAHATI campus google map

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int m=490; // vertical size of the map
const int n=1081; // horizontal size of the map
static int m_ap[n][m];
static int closed_nodes_map[n][m]; // map of closed nodes list (explored)
static int open_nodes_map[n][m];   // map of open nodes list (not explored yet) 
static int dir_map[n][m];          // map of directions
const int dir=8;                   // number of possible directions to go at any position
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance it has already travelled to reach the node
    int priority;  // smaller: higher priority
    int level;
    // priority=level+remaining distance estimate
    public:
        node(int xp, int yp, int d, int p) 
            {xPos=xp; yPos=yp; level=d; priority=p;}
    
        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i) // i: direction
        {
             level+=(dir==8?(i%2==0?10:14):10);
        }
        
        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;         
            d=static_cast<int>(sqrt(xd*xd+yd*yd));            // Euclidian Distance
            return(d);
        }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A* algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart,const int & xFinish, const int & yFinish )
{
    static priority_queue<node> pq[2]; // list of open nodes (not-yet-explored)
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A-star search
    while(!pq[pqi].empty())
    {
        // get the current node with the highest priority from the open list nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop();                                 // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();           
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || m_ap[xdx][ydy]==1 
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(), 
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace d node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx && 
                           pq[pqi].top().getyPos()==ydy))
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pq[pqi].pop(); // remove the wanted node
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

int main(int argc,char** argv)
{

Mat A,A2,A1;
Mat B=imread(argv[1],1);        //Read the iitg_google_map image 

cvtColor(B,A1, CV_RGB2GRAY);   //Convert to grayscale image
A1=255-	A1;

   //Convert grayscale image to binary image
   for(int y=0;y<A1.rows;y++)
   {
       for(int x=0;x<A1.cols;x++) 
        {
           if(A1.at<uchar>(y,x)) {A1.at<uchar>(y,x)=255;}
        }
   }

A1=255-	A1;
distanceTransform(A1,A2,CV_DIST_L2,5 );       //Distance transform
normalize(A2,A2, 0,255, NORM_MINMAX);         //Normalize

//Minimum allowed distance to any obstacle is 30 pixels.
   for(int j=0;j<A2.rows;j++) 
   {
      for(int i=0;i<A2.cols;i++)
      {
           if(A2.at<float>(j,i)<30)   { A2.at<float>(j,i)=0;}
           else                       {A2.at<float>(j,i)=255;}
      }
   }

A2=255-A2;
A2.convertTo(A, CV_8UC1);

int m=1081,n=490;
    
    for(int y=0;y<A.rows;y++)                                    
    {
       for(int x=0;x<A.cols;x++) 
       {
            m_ap[x][y]=(int)A.at<uchar>(y,x)/255; //Assign image matrix to the map m_ap
       }    
    }
      
    //selecting start and finish locations
int xA=29, yA=407, xB=1006, yB=111;
    // get the route
    string route=pathFind(xA, yA, xB, yB);
    // follow the route on the map and display it 
    if(route.length()>0)
    {
        int j; char c;
        int x=xA;
        int y=yA;
        m_ap[x][y]=2;//start
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c); 
            x=x+dx[j];
            y=y+dy[j];
            m_ap[x][y]=2;//assign '2' to the points in the shortest path trajectory
        }
        m_ap[x][y]=2;//finish
    
        // display the map with the route
        for(int y=0;y<A.rows;y++)
        {
            for(int x=0;x<A.cols;x++) 
            {
                   if(m_ap[x][y]==2)  {B.at<Vec3b>(y,x)=255;}
            }
        }
                        
     circle(B,Point(29,407),5,50,1,8,0);              //Draw a small circle at initial posiiton
     circle(B,Point(1006,111),5,50,1,8,0);            //Draw a small circle at final position
     namedWindow("Pathplanning_IITG",WINDOW_NORMAL);  
     imshow("Pathplanning_IITG",B);                   //Display the final image 
     waitKey(0);
     }  
return(0);
}

