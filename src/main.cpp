#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
//#include <windows.h>
//#include <conio.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include <stdexcept>

// Флаг локальной отладки. Вводится при компиляции

#if defined(INTERACTIVE_MODE)  || defined(CLI_FILE_MODE)
    #define DBG(...) printf(__VA_ARGS__)
#else
    #define DBG(...) ((void)0)
#endif

const int N=10000;//points to visit
const int Tmax=15;//skills
int LN=2*(int)(log(N)/log(2)+5);
const int M=500;//offices
const int M_=M*5;
const int WO=10000;//workers
const int M__=500;//how many workers can be on one office
const float pi=3.14159265;

double similar_rad=0.00001;

float initial_summ;//add all the prizes

int Nodes_min=-1;
int Nodes_max=-1;
int gap_treshold;

float Ob_x[N],Ob_y[N];
bool Ob_skill[N][Tmax];
float Ob_weight[N];
float Base_weight[M];
float Base_x[M],Base_y[M];
float Ob_Base_dist[N][M];
int Ob_back[N],Ob_forward[N];//to store a path
int Base_start[M],Base_finish[M];//to store path begin and end
int Base_nodes[M];
int Base_nodes_max[M];
int Base_skills[M];
int Base_Worker[M][M__];
int Base_max[M];
bool Worker_skill[WO][Tmax];
int worker_max=0;
int Ob_back_save[N][3],Ob_forward_save[N][3];//to store a path - saved
int Base_start_save[M][3],Base_finish_save[M][3],Base_max_save[M][3],Base_Worker_save[M][M__][3];//to store path begin and end - saved
int m=0,n=0,wo=0;

bool base_stays_on_cluster=true;

int Ob_freed[N],Ob_freed_max;

float skill_wanna[M][Tmax];

int QTree_big[N],QTree_small[N],QTree_back[N],QTree_Storage[N],QTree_max=0;//QTree data structure to get faster access to only necessary vertices
float QTree_treshold[N];
char QTree_variable[N];
int Storage_max[N],Storage_max_max=0;
int* Storage[N];
int misses;
bool increased_accuracy=false;

bool Steiner_moveable[M_];//Steiner data structure to get convexity always
float Steiner_x[M_],Steiner_y[M_];
float Steiner_x_save[M_][3],Steiner_y_save[M_][3];
int SteinerNeighbors[M_][M_],SteinerNeighborsMax[M_];
int SteinerNeighbors_R[M_][M_],SteinerNeighbors_L[M_][M_];
bool Steiner_do_move_deg3=false;
int ms=0;

int SteinerConstraints_A[M_][M_],SteinerConstraints_B[M_][M_],SteinerConstraints_max[M_];//For constraints to get the gradient projection
float Steiner_SAT;

int Wanna[N],Current[N];//Which cluster do we want vertice to belong, Which is it now

double aim_x,aim_y;

float fun[N];

const int screen_x=50,screen_y=50;

char screen[screen_x][screen_y][3],screen_[screen_x][screen_y][3];

float global_rad_x,global_rad_y;

float coeffs[10],coeffs_min[10],coeffs_max[10];
const int qu_coeffs=100;
float coeffs_density[10][qu_coeffs];//quanting

int time_final;

bool is_time_still() {
	if (time(0)>time_final) {return false;}
	return true;
}

int rand_huge() {
	int ans=0,d=1;
	for (int i=0;i<20;i++) {
		if (rand()%2==0) {
			ans+=d;
		}
		d*=2;
	}
	return ans;
}

float f1(float r) {
	return 1;
}

void movepos( short x, short y ) { 
    /*HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE) ; 
    COORD position = { x, y } ; 
     
    SetConsoleCursorPosition( hStdout, position ) ; /**/
}  

void ListStorage() {
	DBG("Tree:\n");
	for (int i=0;i<QTree_max;i++) {
		DBG("%d:%d < %c < %d  - %d\n",i,QTree_small[i],QTree_variable[i],QTree_big[i],QTree_Storage[i]);
	}
	DBG("Storages:\n");
	for (int i=0;i<Storage_max_max;i++) {
		DBG("%d:",i);
		for (int j=0;j<Storage_max[i];j++) {
			DBG(" %d ",Storage[i][j]);
		}
		DBG("\n");
	}
	
}

void ListSteiner() {
	DBG("vertex:\n");
	for (int i=0;i<ms;i++) {
		DBG("%2d :",i);
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			DBG("%2d [%d %d] ",SteinerNeighbors[i][j],SteinerNeighbors_R[i][j],SteinerNeighbors_L[i][j]);
		}DBG(":\n");
	}
	DBG("coords:\n");
	for (int i=0;i<ms;i++) {
		DBG("%d : %.2f %.2f\n",i,Steiner_x[i],Steiner_y[i]);
	}
}

void Draw_new(int dx,int dy) {
	for (int y=0;y<screen_y;y+=dy) {
		for (int x=0;x<screen_x;x+=dx) {
			if (screen[x][y][0]!=screen_[x][y][0]) {
				movepos(x/dx,y/dy+1);
				DBG("%c",screen[x][y][0]);
				screen_[x][y][0]=screen[x][y][0];
			}
		}
		for (int x=0;x<screen_x;x+=dx) {
			if (screen[x][y][1]!=screen_[x][y][1]) {
				movepos(x/dx+screen_x/dx+1,y/dy+1);
				DBG("%c",screen[x][y][1]);
				screen_[x][y][1]=screen[x][y][1];
			}
		}
	}
}

void Draw(int dx,int dy) {
	for (int y=0;y<screen_y;y+=dy) {
		for (int x=0;x<screen_x;x+=dx) {
			DBG("%c",screen[x][y][0]);
		}
		DBG(";");
		for (int x=0;x<screen_x;x+=dx) {
			DBG("%c",screen[x][y][1]);
		}
		DBG(";\n");
	}DBG("\n");
}

void draw_routes() {
	int sk[Tmax];
	
	for (int i=0;i<m-4;i++) {
		DBG("Base %d:",i);
		int g;
		
		for (int ii=0;ii<Tmax;ii++) {sk[ii]=0;}
		
		g=Base_start[i];
		while ((g!=Base_finish[i])&&(g!=-1)) {
			//DBG("-[%d %d]-",g,Current[g]);
			DBG("[%d]",g);
			for (int ii=0;ii<Tmax;ii++) {if (Ob_skill[g][ii]==true) {sk[ii]++;}}
			g=Ob_forward[g];
		}
		if (g!=-1) {
			//DBG("-[%d %d]-",g,Current[g]);
			for (int ii=0;ii<Tmax;ii++) {if (Ob_skill[Base_finish[i]][ii]==true) {sk[ii]++;}}
			DBG("[%d]",g);
			if (g==Base_finish[i]) {DBG(";");}
			for (int ii=0;ii<Tmax;ii++) {
				DBG("%d:",sk[ii]);
			}
		} else {
			DBG("|");
		}
		DBG("                          \n");
		
		DBG("workers:");
		for (int j=0;j<Base_max[i];j++) {
			DBG(" [%d]",Base_Worker[i][j]);
			for (int k=0;k<Tmax;k++) {
				DBG("%c",Worker_skill[Base_Worker[i][j]][k]+'A');
			}
		}
		DBG(";                          \n");
		
	}
}

void routes_save(int mode) {
	for (int i=0;i<n;i++) {
		Ob_back_save[i][mode]=Ob_back[i];
		Ob_forward_save[i][mode]=Ob_forward[i];
	}
	for (int i=0;i<m;i++) {
		Base_max_save[i][mode]=Base_max[i];
		Base_start_save[i][mode]=Base_start[i];
		Base_finish_save[i][mode]=Base_finish[i];
		
		for (int j=0;j<Base_max[i];j++) {
			Base_Worker_save[i][j][mode]=Base_Worker[i][j];
		}
	}
}

void routes_load(int mode) {
	for (int i=0;i<n;i++) {
		Ob_back[i]=Ob_back_save[i][mode];
		Ob_forward[i]=Ob_forward_save[i][mode];
	}
	for (int i=0;i<m;i++) {
		Base_max[i]=Base_max_save[i][mode];
		Base_start[i]=Base_start_save[i][mode];
		Base_finish[i]=Base_finish_save[i][mode];
		for (int j=0;j<Base_max[i];j++) {
			Base_Worker[i][j]=Base_Worker_save[i][j][mode];
		}
	}
}

void Steiner_save(int mode) {
	for (int i=0;i<ms;i++) {
		Steiner_x_save[i][mode]=Steiner_x[i];
		Steiner_y_save[i][mode]=Steiner_y[i];
	}
}

void Steiner_load(int mode) {
	for (int i=0;i<ms;i++) {
		Steiner_x[i]=Steiner_x_save[i][mode];
		Steiner_y[i]=Steiner_y_save[i][mode];
	}
}

void Steiner_generate() {
	ms=0;
	double Base_x_[M*2],Base_y_[M*2];
	double dx[M*2],dy[M],mx[M*2],my[M*2],ddx,ddy,ddr,ddx_,ddy_,ddr_,angle[M*2];
	int imin,j,j_,a,b,vlink[M*2],A,B;
	double vmax,t;
	double vx[M*2],vy[M*2];
	bool valid[M*2],isit;
	
	for (int i=0;i<m;i++) {//doing for i
		Base_x_[i]=Base_x[i]+(rand_huge()%100000/100000.0-0.5);
		Base_y_[i]=Base_y[i]+(rand_huge()%100000/100000.0-0.5);
	}
	
	for (int i=0;i<m-4;i++) {//doing for i
		
		for (int j=0;j<m;j++) {valid[j]=1;}
		
		for (int j=0;j<m;j++) {//generating planes
			if (j!=i) {
				dx[j]=Base_x_[i]-Base_x_[j];
				dy[j]=Base_y_[i]-Base_y_[j];
				mx[j]=(Base_x_[i]+Base_x_[j])/2.0;
				my[j]=(Base_y_[i]+Base_y_[j])/2.0;
				
				//[mx+tdy-xi]2+[my-tdx-yi]2
				//tt[dxdx+dydy]+2t[ dy[mx-xi]-dx[my-yi] ]+[mx-xi]2+[my-yi]2
				//t[dxdx+dydy]=-[ dy[mx-xi]-dx[my-yi] ]
				t=-( dy[j]*(mx[j]-Base_x_[i])-dx[j]*(my[j]-Base_y_[i]) ) / ( dx[j]*dx[j]+dy[j]*dy[j] );
				
				mx[j]+=t*dy[j];
				my[j]-=t*dx[j];
				
				ddx=Base_x_[j]-Base_x_[i];
				ddy=Base_y_[j]-Base_y_[i];
				ddr=ddx*ddx+ddy*ddy;
				
				angle[j]=acos(ddx/sqrt(ddr));
				if (ddy<0) {angle[j]=2*pi-angle[j];}
			}
		}
		dx[i]=dx[m-1];//copy m-1 to i to avoid holes
		dy[i]=dy[m-1];
		mx[i]=mx[m-1];
		my[i]=my[m-1];
		angle[i]=angle[m-1];
		
		//sort them, further maybe replace with something better
		for (int j=0;j<m-1;j++) {
			for (int k=0;k<m-2;k++) {
				if (angle[k]>angle[k+1]) {
					ddr=dx[k];dx[k]=dx[k+1];dx[k+1]=ddr;
					ddr=dy[k];dy[k]=dy[k+1];dy[k+1]=ddr;
					ddr=mx[k];mx[k]=mx[k+1];mx[k+1]=ddr;
					ddr=my[k];my[k]=my[k+1];my[k+1]=ddr;
					ddr=angle[k];angle[k]=angle[k+1];angle[k+1]=ddr;
				}
			}
		}
		
		
		for (int j=0;j<m-1;j++) {
			valid[j]=1;
			j_=j-1;
			if (j_<0) {j_+=m;}
			if (angle[j]==angle[j-1]) {//two lines are collinear, disable furthest
				if ( (mx[j]-Base_x_[i])*(mx[j]-Base_x_[i])+(my[j]-Base_y_[i])*(my[j]-Base_y_[i]) > (mx[j_]-Base_x_[i])*(mx[j_]-Base_x_[i])+(my[j_]-Base_y_[i])*(my[j_]-Base_y_[i]) ) {
					valid[j]=0;
				} else {
					valid[j-1]=0;
				}
			}
		}
		
		isit=true;
		while (isit==true) {
			
			a=0;
			
			j=0;
			
			while (valid[j]==0) {
				j++;
				if (j>=m-1) {j-=m-1;}
			}
			
			b=j;
			j_=-1;
			
			while (j_!=b) {
				
				j_=j+1;
				if (j_>=m-1) {j_-=m-1;}
				while (valid[j_]==0) {
					j_++;
					if (j_>=m-1) {j_-=m-1;}
				}
				
				//connect j and j_ at a point
				
				/*
				(x-mx[j])dx[j]+(y-my[j])dy[j]=0
				(x-mx[j_])dx[j_]+(y-my[j_])dy[j_]=0
				
				x*dx[j]+y*dy[j]=mx[j]*dx[j]+dy[j]*my[j]
				x*dx[j_]+y*dy[j_]=mx[j_]*dx[j_]+dy[j_]*my[j_]
				
				
				x*dx[j_]*dx[j]+y*dy[j]*dx[j_]=mx[j]*dx[j]*dx[j_]+dy[j]*my[j]*dx[j_]
				x*dx[j_]*dx[j]+y*dy[j_]*dx[j]=mx[j_]*dx[j_]*dx[j]+dy[j_]*my[j_]*dx[j]
				
				x*dx[j]*dy[j_]+y*dy[j_]*dy[j]=mx[j]*dx[j]*dy[j_]+dy[j]*my[j]*dy[j_]
				x*dx[j_]*dy[j]+y*dy[j_]*dy[j]=mx[j_]*dx[j_]*dy[j]+dy[j_]*my[j_]*dy[j]
				
				
				y*(dy[j]*dx[j_]-dy[j_]*dx[j])=(mx[j]*dx[j]+dy[j]*my[j])*dx[j_]-(mx[j_]*dx[j_]+dy[j_]*my[j_])*dx[j]
				x*(dx[j]*dy[j_]-dx[j_]*dy[j])=(mx[j]*dx[j]+dy[j]*my[j])*dy[j_]-(mx[j_]*dx[j_]+dy[j_]*my[j_])*dy[j]
				*/
				
				vx[a]=(double)((mx[j]*dx[j]+dy[j]*my[j])*dy[j_] - (mx[j_]*dx[j_]+dy[j_]*my[j_])*dy[j]) / (double)(dx[j]*dy[j_] - dx[j_]*dy[j]);
				vy[a]=(double)((mx[j]*dx[j]+dy[j]*my[j])*dx[j_] - (mx[j_]*dx[j_]+dy[j_]*my[j_])*dx[j]) / (double)(dy[j]*dx[j_] - dy[j_]*dx[j]);
				vlink[a]=j_;
				a++;
				j=j_;
				
			}
			
			isit=false;
			for (j=0;j<a;j++) {
				j_=j-1;
				if (j_<0) {j_+=a;}
				while ( (vx[j_]-mx[vlink[j]])*dx[vlink[j]] + (vy[j_]-my[vlink[j]])*dy[vlink[j]] < -similar_rad ) {
					if (valid[vlink[j_]]==1) {isit=true;}
					valid[vlink[j_]]=0;
					j_--;
					if (j_<0) {j_+=a;}
				}
			}
		}
		
		
		//now updage Steiner graph
		
		for (int j=0;j<a;j++) {
			//adding A=j vertice
			imin=ms;
			for (int k=0;k<ms;k++) {
				ddx=vx[j]-Steiner_x[k];
				ddy=vy[j]-Steiner_y[k];
				if (ddx*ddx+ddy*ddy<=similar_rad) {
					imin=k;
				}
			}
			if (imin==ms) {
				Steiner_x[ms]=vx[j];
				Steiner_y[ms]=vy[j];
				A=ms;
				ms++;
			} else {
				A=imin;
			}
			
			//adding B=j-1 vertice
			j_=j-1;
			if (j_<0) {j_+=a;}
			
			imin=ms;
			for (int k=0;k<ms;k++) {
				ddx=vx[j_]-Steiner_x[k];
				ddy=vy[j_]-Steiner_y[k];
				if (ddx*ddx+ddy*ddy<=similar_rad) {
					imin=k;
				}
			}
			if (imin==ms) {
				Steiner_x[ms]=vx[j_];
				Steiner_y[ms]=vy[j_];
				B=ms;
				ms++;
			} else {
				B=imin;
			}
			
			//now add an edge between them
			
			//to A
			isit=true;
			for (int k=0;k<SteinerNeighborsMax[A];k++) {
				if (SteinerNeighbors[A][k]==B) {
					isit=false;
					SteinerNeighbors_R[A][k]=i;
				}
			}
			if (isit==true) {
				SteinerNeighbors[A][SteinerNeighborsMax[A]]=B;
				SteinerNeighbors_R[A][SteinerNeighborsMax[A]]=i;
				SteinerNeighbors_L[A][SteinerNeighborsMax[A]]=-1;
				SteinerNeighborsMax[A]++;
			}
			
			//to B
			isit=true;
			for (int k=0;k<SteinerNeighborsMax[B];k++) {
				if (SteinerNeighbors[B][k]==A) {
					isit=false;
					SteinerNeighbors_L[B][k]=i;
				}
			}
			if (isit==true) {
				SteinerNeighbors[B][SteinerNeighborsMax[B]]=A;
				SteinerNeighbors_L[B][SteinerNeighborsMax[B]]=i;
				SteinerNeighbors_R[B][SteinerNeighborsMax[B]]=-1;
				SteinerNeighborsMax[B]++;
			}
		}
		
		//now updage the screen
		
		for (int x=-49;x<50;x++) {
			for (int y=-49;y<50;y++) {
				bool dodo=true;
				if (i>=m-4) {dodo=false;}
				for (int j=1;j<a;j++) {
					if ((vx[j]-vx[j-1])*(y-vy[j-1])-(vy[j]-vy[j-1])*(x-vx[j-1])<0) {
						dodo=false;
					}
				}
				if ((vx[0]-vx[a-1])*(y-vy[a-1])-(vy[0]-vy[a-1])*(x-vx[a-1])<0) {
					dodo=false;
				}
				if (dodo==true) {
					screen[x+50][y+50][2]='a'+i;
				}
				if ((x-Base_x[i])*(x-Base_x[i])+(y-Base_y[i])*(y-Base_y[i])<=1) {
					//screen[x+50][y+50]='A'+i;
					screen[x+50][y+50][2]='_';
				}
			}
		}
		
	}
	
	//now mark all who has access to the outer space, they can not be mover while doing variations
	for (int i=0;i<ms;i++) {
		Steiner_moveable[i]=true;
		for (int j=0;(j<SteinerNeighborsMax[i])&&(Steiner_moveable[i]==true);j++) {
			if ((SteinerNeighbors_R[i][j]==-1)||(SteinerNeighbors_L[i][j]==-1)) {
				if ((Steiner_do_move_deg3==false)||(SteinerNeighborsMax[i]==2)) {Steiner_moveable[i]=false;}
			}
		}
	}
}

void determine_class_prepare() {
	double xa,ya,xb,yb,rad,t,aimx,aimy;
	int angle_max_max=50;
	double* angle=(double*)(malloc(sizeof(double)*angle_max_max));
	int angle_max=0;
	
	for (int i=0;i<ms;i++) {
		//finding out the best angle
		xa=Steiner_x[i];
		ya=Steiner_y[i];
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			xb=Steiner_x[SteinerNeighbors[i][j]];
			yb=Steiner_y[SteinerNeighbors[i][j]];
			if ((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb)>0.00001) {
				angle[angle_max]=acos((xa-xb)/((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb)));
				if (ya-yb<0) {angle[angle_max]=2*pi-angle[angle_max];}
				angle_max++;
				if (angle_max>angle_max_max/2) {
					angle_max_max*=2;
					angle=(double*)(realloc(angle,sizeof(double)*angle_max_max));
				}
			}
		}
	}
	
	for (int i=0;i<angle_max;i++) {
		for (int j=0;j<angle_max-1;j++) {
			if (angle[j]>angle[j+1]) {
				xa=angle[j];angle[j]=angle[j+1];angle[j+1]=xa;
			}
		}
	}
	rad=0;
	for (int i=1;i<angle_max;i++) {
		t=fabs(angle[i]-angle[i-1]);
		if (t>rad) {
			rad=t;
			aimx=(angle[i]+angle[i-1])/2;
		}
	}
	t=fabs(angle[0]-angle[angle_max-1]);
	if (t>rad) {
		rad=t;
		aimx=(angle[0]+angle[angle_max-1])/2;
	}
	aim_y=sin(aimx);
	aim_x=cos(aimx);
}

int determine_class(float x,float y) {
	double xa,ya,xb,yb,dx,dy,aimx=aim_x,aimy=aim_y,t,radmin=-1,rad;
	int ans=-1;
	
	for (int i=0;i<ms;i++) {
		//reaching for line [x+t*aimy,y-t*aimx]:t>=0, trying to touch it closer to [x,y] as we can, then determine class by those line
		xa=Steiner_x[i];
		ya=Steiner_y[i];
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			xb=Steiner_x[SteinerNeighbors[i][j]];
			yb=Steiner_y[SteinerNeighbors[i][j]];
			dx=xb-xa;
			dy=yb-ya;
			//<[xa+t*dx-x,ya+t*dy-y];[aimx,aimy]> = 0
			//[xa+t*dx-x]*aimx+[ya+t*dy-y]*aimy = 0
			//t = - ([xa-x]*aimx + [ya-y]*aimy) / [dx*aimx+dy*aimy]
			t = - ((xa-x)*aimx + (ya-y)*aimy) / (dx*aimx+dy*aimy);
			if ((t>=0)&&(t<=1)) {
				dx=xa+dx*t-x;
				dy=ya+dy*t-y;
				rad=dx*dx+dy*dy;
				if ((radmin==-1)||(rad<radmin)) {
					radmin=rad;
					dx=xb-xa;
					dy=yb-ya;
					//<[x-xa,y-ya];[-dy,dx]> >= 0
					if ( -(x-xa)*dy+(y-ya)*dx > 0.00001 ) {
						ans=SteinerNeighbors_L[i][j];
					}
					if ( -(x-xa)*dy+(y-ya)*dx <- 0.00001 ) {
						ans=SteinerNeighbors_R[i][j];
					}
				}
			}
		}
	}
	return ans;
}

void determine_class_forall() {
	int aa;
	for (int i=0;i<n;i++) {
		aa=-2;
		while (aa==-2) {aa=determine_class(Ob_x[i],Ob_y[i]);}
		Current[i]=aa;
	}
}

void ScreenClear() {
	for (int y=0;y<screen_y;y++) {
		for (int x=0;x<screen_x;x++) {
			screen_[x][y][0]=' ';
			screen_[x][y][1]=' ';
		}
	}
}

void ScreenPrepare(int dx,int dy,int movx,int movy, bool do_only_points, char screen_number) {
	int aa,x,y;
	bool isit;
	for (int x=0;x<screen_x;x++) {
		for (int y=0;y<screen_y;y++) {
			screen[x][y][screen_number]=' ';
		}
	}
	
	if (do_only_points==false) {
		for (int i=0;i<n;i++) {
			x=(int)(Ob_x[i]/(float)dx+screen_x/2-movx/(float)dx);
			y=(int)(Ob_y[i]/(float)dy+screen_y/2-movy/(float)dy);
			if ((x>=0)&&(x<screen_x)&&(y>=0)&&(y<screen_y)) {
				aa=-2;
				while (aa==-2) {aa=determine_class(Ob_x[i],Ob_y[i]);}
				
				aa=aa+'0';
				while (aa>'0'+30) {aa-=30;}
				while (aa<'0'-10) {aa+=10;}
				
				if (aa=='0'-1) {
					aa=' ';
				}
				
				screen[x][y][screen_number]=aa;
			}
		}
	}
	
	for (int i=0;i<m-4;i++) {
		x=(int)(Base_x[i]/(float)dx+screen_x/2-movx/(float)dx);
		y=(int)(Base_y[i]/(float)dy+screen_y/2-movy/(float)dy);
		if ((x>=0)&&(x<screen_x)&&(y>=0)&&(y<screen_y)) {
			screen[x][y][screen_number]='@';
		}
	}
	
	if (do_only_points==true) {
		for (int x=0;x<screen_x;x++) {
			for (int y=0;y<screen_y;y++) {
				aa=-2;
				while (aa==-2) {aa=determine_class((x-screen_x/2)*dx+movx,(y-screen_y/2)*dy+movy);}
				
				aa=aa+'0';
				while (aa>'0'+30) {aa-=30;}
				while (aa<'0'-10) {aa+=10;}
				
				if (aa=='0'-1) {
					aa=' ';
				}
				
				screen[x][y][screen_number]=aa;
			}
		}
	}
}

void draw_a_line(float xa,float ya,float xb,float yb,char screen_number) {
	float T;
	int x,y;
	char ch;
	if ((xa<0)&&(xb>0)) {
		T=(0-xb)/(xa-xb);
		xa=xb+(xa-xb)*T;
		ya=yb+(ya-yb)*T;
	}
	if ((xa>0)&&(xb<0)) {
		T=(0-xa)/(xb-xa);
		xb=xa+(xb-xa)*T;
		yb=ya+(yb-ya)*T;
	}
	if ((ya<0)&&(yb>0)) {
		T=(0-yb)/(ya-yb);
		xa=xb+(xa-xb)*T;
		ya=yb+(ya-yb)*T;
	}
	if ((ya>0)&&(yb<0)) {
		T=(0-ya)/(yb-ya);
		xb=xa+(xb-xa)*T;
		yb=ya+(yb-ya)*T;
	}
	
	if ((ya-yb)*(xa-xb)<0) {
		ch='/';
	} else {
		ch='\\';
	}
	
	if (abs(ya-yb)>abs(xa-xb)) {//dy is bigger
		T=abs(ya-yb)+10;
		if (abs(ya-yb)>abs(xa-xb)*2) {
			ch='|';
		}
	} else {//dx is bigger
		T=abs(xa-xb)+10;
		if (abs(xa-xb)>abs(ya-yb)*2) {
			ch='-';
		}
	}
	for (int t=0;t<=T;t++) {
		x=xa+t*(xb-xa)/T;
		y=ya+t*(yb-ya)/T;
		if ((x>=0)&&(x<screen_x)&&(y>=0)&&(y<screen_y)&&(screen[x][y][screen_number]==' ')) {
			screen[x][y][screen_number]=ch;
		}
	}
}

void ScreenPrepareRoutes(int dx,int dy,int movx,int movy, char screen_number) {
	int g,g_;
	bool isit;
	for (int i=0;i<m-4;i++) {
		
		if (Base_start[i]!=-1) {
			draw_a_line(Base_x[i]/dx+screen_x/2-movx/dx,Base_y[i]/dy+screen_y/2-movy/dy,Ob_x[Base_start[i]]/dx+screen_x/2-movx/dx,Ob_y[Base_start[i]]/dy+screen_y/2-movy/dy,screen_number);
		}
		
		g=Base_start[i];
		while ((g!=Base_finish[i])&&(g!=-1)) {
			g_=g;
			g=Ob_forward[g];
			if ((g!=-1)&&(g_!=-1)) {draw_a_line(Ob_x[g]/dx+screen_x/2-movx/dx,Ob_y[g]/dy+screen_y/2-movy/dy,Ob_x[g_]/dx+screen_x/2-movx/dx,Ob_y[g_]/dy+screen_y/2-movy/dy,screen_number);}
		}
		
		if (Base_finish[i]!=-1) {
			draw_a_line(Base_x[i]/dx+screen_x/2-movx/dx,Base_y[i]/dy+screen_y/2-movy/dy,Ob_x[Base_finish[i]]/dx+screen_x/2-movx/dx,Ob_y[Base_finish[i]]/dy+screen_y/2-movy/dy,screen_number);
		}
		
	}
}

void ScreenSteiner(int screen_number) {
	for (int x=0;x<screen_x;x++) {
		for (int y=0;y<screen_y;y++) {
			screen[x][y][screen_number]=-1;
		}
	}
	for (int x=0;x<screen_x;x++) {
		for (int y=0;y<screen_y;y++) {
			for (int i=0;i<ms;i++) {
				if (fabs(x-screen_x/2+Steiner_x[i])+fabs(y-screen_y/2+Steiner_y[i])<1) {
					screen[x][y][screen_number]='a'+i;
				}
			}
		}
	}
}

float median_storage(int s,char variable) {
	
	float tt;
	
	for (int i=0;i<Storage_max[s];i++) {
		if (variable=='x') {fun[i]=Ob_x[Storage[s][i]];}
		if (variable=='y') {fun[i]=Ob_y[Storage[s][i]];}
	}
	
	for (int i=0;i<=Storage_max[s];i++) {
		for (int j=0;j<Storage_max[s]-1;j++) {
			if (fun[j]>fun[j+1]) {
				tt=fun[j];
				fun[j]=fun[j+1];
				fun[j+1]=tt;
			}
		}
	}
	
	tt=fun[Storage_max[s]/2+1];
	
	misses=0;
	for (int i=0;i<=Storage_max[s];i++) {
		if (fun[i]==tt) {misses++;}
	}
	
	return tt;
}

void QTree_destroy() {//to avoid memory leak
	for (int i=0;i<Storage_max_max;i++) {
		delete(Storage[i]);
	}
}

void Qtree_generate() {
	float medx,medy;
	int missx,missy,run=0;
	
	char count_x[N],count_y[N],count_max=2;//balanced trees
	count_x[0]=0;
	count_y[0]=0;
	
	//generating the first node
	QTree_Storage[0]=0;
	QTree_max=3;
	Storage_max[0]=n;
	Storage[0]=(int*)malloc(sizeof(int)*n);
	Storage[1]=(int*)malloc(sizeof(int)*n);
	Storage_max_max=2;
	
	for (int i=0;i<n;i++) {
		Storage[0][i]=i;
	}
	
	medx=median_storage(0,'x');
	missx=misses;
	
	medy=median_storage(0,'y');
	missy=misses;
	
	if (missx<=missy) {
		QTree_variable[0]='x';
		QTree_treshold[0]=medx;
	} else {
		QTree_variable[0]='y';
		QTree_treshold[0]=medy;
	}
	
	QTree_big[0]=1;
	QTree_small[0]=2;
	Storage_max[1]=0;
	
	QTree_variable[1]='l';
	QTree_variable[2]='l';
	
	QTree_back[0]=-1;
	QTree_back[1]=0;
	QTree_back[2]=0;
	
	QTree_Storage[1]=QTree_Storage[0];
	QTree_Storage[2]=1;
	
	if (QTree_variable[0]=='x') {
		count_x[QTree_big[0]]=count_x[0]+1;
		count_y[QTree_big[0]]=0;
		
		count_x[QTree_small[0]]=count_x[0]+1;
		count_y[QTree_small[0]]=0;
	}
	if (QTree_variable[0]=='y') {
		count_y[QTree_big[0]]=count_y[0]+1;
		count_x[QTree_big[0]]=0;
		
		count_y[QTree_small[0]]=count_y[0]+1;
		count_x[QTree_small[0]]=0;
	}
	
	for (int i=0;i<Storage_max[0];i++) {
		//storage->Q_big, new storage->Q_small, both of them -> Q_bog & Q_small
		if (
			((Ob_x[Storage[0][i]]<QTree_treshold[0])&&(QTree_variable[0]=='x'))||
			((Ob_y[Storage[0][i]]<QTree_treshold[0])&&(QTree_variable[0]=='y'))
		) {
			Storage[1][Storage_max[1]]=Storage[0][i];//copy to Storage_small
			Storage_max[1]++;
			Storage[0][i]=Storage[0][Storage_max[0]-1];//delete from Storage_big
			Storage_max[0]--;
		}
		if (increased_accuracy==true) {
			if (
				((Ob_x[Storage[0][i]]==QTree_treshold[0])&&(QTree_variable[0]=='x'))||
				((Ob_y[Storage[0][i]]==QTree_treshold[0])&&(QTree_variable[0]=='y'))
			) {
				Storage[1][Storage_max[1]]=Storage[0][i];//copy to Storage_small
				Storage_max[1]++;
			}
		}
	}
	
	int n_min=log(n)/log(2)*2;//be careful with this, can eat RAM!!!
	
	run++;
	while (run<QTree_max) {
		
		if (Storage_max[QTree_Storage[run]]>n_min) {
			
			medx=median_storage(QTree_Storage[run],'x');
			missx=misses;
			
			medy=median_storage(QTree_Storage[run],'y');
			missy=misses;
			
			if (missx<=missy) {
				QTree_variable[run]='x';
				QTree_treshold[run]=medx;
			} else {
				QTree_variable[run]='y';
				QTree_treshold[run]=medy;
			}
			
			QTree_big[run]=QTree_max;
			QTree_small[run]=QTree_max+1;
			Storage_max[Storage_max_max]=0;
			Storage[Storage_max_max]=(int*)malloc(sizeof(int)*Storage_max[QTree_Storage[run]]);
			
			QTree_Storage[QTree_big[run]]=QTree_Storage[run];
			QTree_Storage[QTree_small[run]]=Storage_max_max;
			
			QTree_variable[QTree_big[run]]='l';
			QTree_variable[QTree_small[run]]='l';
			
			QTree_back[QTree_big[run]]=run;
			QTree_back[QTree_small[run]]=run;
			
			QTree_max+=2;
			Storage_max_max++;
			
			if ((QTree_variable[run]=='x')&&(count_x[run]>=count_max)) {//forcing balance between x and y
				QTree_variable[run]='y';
			}
			if ((QTree_variable[run]=='y')&&(count_y[run]>=count_max)) {
				QTree_variable[run]='x';
			}
			
			if (QTree_variable[run]=='x') {
				count_x[QTree_big[run]]=count_x[run]+1;
				count_y[QTree_big[run]]=0;
				
				count_x[QTree_small[run]]=count_x[run]+1;
				count_y[QTree_small[run]]=0;
			}
			if (QTree_variable[run]=='y') {
				count_y[QTree_big[run]]=count_y[run]+1;
				count_x[QTree_big[run]]=0;
				
				count_y[QTree_small[run]]=count_y[run]+1;
				count_x[QTree_small[run]]=0;
			}
			
			for (int i=0;i<Storage_max[QTree_Storage[run]];i++) {
				//storage->Q_big, new storage->Q_small, both of them -> Q_bog & Q_small
				if (
					((Ob_x[Storage[QTree_Storage[run]][i]]<QTree_treshold[run])&&(QTree_variable[run]=='x'))||
					((Ob_y[Storage[QTree_Storage[run]][i]]<QTree_treshold[run])&&(QTree_variable[run]=='y'))
				) {
					Storage[QTree_Storage[QTree_small[run]]][Storage_max[QTree_Storage[QTree_small[run]]]]=Storage[QTree_Storage[run]][i];//copy to Storage_small
					Storage_max[QTree_Storage[QTree_small[run]]]++;
					Storage[QTree_Storage[QTree_big[run]]][i]=Storage[QTree_Storage[QTree_big[run]]][Storage_max[QTree_Storage[QTree_big[run]]]-1];//delete from Storage_big
					Storage_max[QTree_Storage[QTree_big[run]]]--;
				}
				if (increased_accuracy==true) {
					if (//in case you need increase the accuracy
						((Ob_x[Storage[QTree_Storage[run]][i]]==QTree_treshold[run])&&(QTree_variable[run]=='x'))||
						((Ob_y[Storage[QTree_Storage[run]][i]]==QTree_treshold[run])&&(QTree_variable[run]=='y'))
					) {
						Storage[QTree_Storage[QTree_small[run]]][Storage_max[QTree_Storage[QTree_small[run]]]]=Storage[QTree_Storage[run]][i];//copy to Storage_small
						Storage_max[QTree_Storage[QTree_small[run]]]++;
					}
				}
			}
			
		}
		
		run++;
		
	}
	
}

void Steiner_prepare_constraints() {//prepare constraints for Steiner to avoid collapsing the tiles (angle stays always <=pi/2)
	int j_,k_,tt,k0,k1;
	float angle[M_],dx,dy,gx,gy;
	
	for (int i=0;i<ms;i++) {//prepare neighbors of each vertice
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			j_=SteinerNeighbors[i][j];
			dx=Steiner_x[j_]-Steiner_x[i];
			dy=Steiner_y[j_]-Steiner_y[i];
			angle[j]=acos(dx/sqrt(dx*dx+dy*dy));
			if (dy<0) {angle[j]=2*pi-angle[j];}
		}
		//now sort them by angle
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			for (int k=0;k<SteinerNeighborsMax[i]-1;k++) {
				if (angle[k]>angle[k+1]) {
					dx=angle[k];angle[k]=angle[k+1];angle[k+1]=dx;
					j_=SteinerNeighbors[i][k];SteinerNeighbors[i][k]=SteinerNeighbors[i][k+1];SteinerNeighbors[i][k+1]=j_;
					j_=SteinerNeighbors_R[i][k];SteinerNeighbors_R[i][k]=SteinerNeighbors_R[i][k+1];SteinerNeighbors_R[i][k+1]=j_;
					j_=SteinerNeighbors_L[i][k];SteinerNeighbors_L[i][k]=SteinerNeighbors_L[i][k+1];SteinerNeighbors_L[i][k+1]=j_;
				}
			}
		}
	}//now we are ready to find the 3 edge decomposition to find the convexity constraints for variations
	
	for (int i=0;i<ms;i++) {
		SteinerConstraints_max[i]=0;
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			j_=SteinerNeighbors[i][j];
			SteinerConstraints_max[i]=0;
			
			k_=0;
			while (SteinerNeighbors[j_][k_]!=i) {k_++;}//found it, now take one lefter and one righter
			tt=SteinerNeighborsMax[j_];
			
			k0=k_-1;
			k1=k_+1;
			
			if (k0<0) {k0+=tt;}
			if (k1>=tt) {k1-=tt;}
			
			k0=SteinerNeighbors[j_][k0];
			k1=SteinerNeighbors[j_][k1];
			
			//add k_-1
			SteinerConstraints_A[i][SteinerConstraints_max[i]]=j_;
			SteinerConstraints_B[i][SteinerConstraints_max[i]]=k0;
			SteinerConstraints_max[i]++;
			
			if (k0==k1) {
				//after reduction the vertice has degree 2 (one edge was infinite and we have got rid of it)
				//now we need to understand what its order - does it go a-b or b-a
				//assume it goes like a=j_,b=k0 and try to see is it true
				dx=Steiner_x[k0]-Steiner_x[j_];
				dy=Steiner_y[k0]-Steiner_y[j_];
				gx=Steiner_x[i]-Steiner_x[j_];
				gy=Steiner_y[i]-Steiner_y[j_];
				
				if ( -dy*gx+dx*gy < 0 ) {
					//we were wrong, swap them
					SteinerConstraints_A[i][SteinerConstraints_max[i]-1]=k0;
					SteinerConstraints_B[i][SteinerConstraints_max[i]-1]=j_;
				}
			} else {
				//add k_+1
				SteinerConstraints_A[i][SteinerConstraints_max[i]]=k1;
				SteinerConstraints_B[i][SteinerConstraints_max[i]]=j_;
				SteinerConstraints_max[i]++;
			}
		}
	}
	
	//now to take the constraint you have to see the followint thing for constraint number k
	//dy=Steiner_y[SteinerConstraints_B[i][k]] - Steiner_y[SteinerConstraints_A[i][k]] 
	//dx=Steiner_x[SteinerConstraints_B[i][k]] - Steiner_x[SteinerConstraints_A[i][k]] 
	//gx=x - Steiner_x[SteinerConstraints_A[i][k]] 
	//gy=y - Steiner_y[SteinerConstraints_A[i][k]] 
	//<[-dy,dx];[gx,gy]> >= 0
}

void Steiner_variate(float initial_speed,float stochastics,int T,int iter_base) {
	float grad_x[M_],grad_y[M_];
	bool doupdate[N];
	int recall_a,recall_b,a,b;
	int i,j,k,iter;
	float minr=-1,r,xa,ya,xb,yb,t,mass_a,mass_b,dx,dy,gx,gy,vx,vy;
	float step=1;
	bool stilldo;
	for (int tim=0;(tim<T)&&(is_time_still()==true);tim++) {//variating Steiner vertices T times
		
		Steiner_SAT=0;
		
		for (int i=0;i<n;i++) {
			doupdate[i]=false;
		}
		
		//getting all the gradients firstly
		for (i=0;i<ms;i++) {
			grad_x[i]=0;
			grad_y[i]=0;
			
			grad_x[i]=(rand_huge()%2000/1000.0-1)*stochastics;
			grad_y[i]=(rand_huge()%2000/1000.0-1)*stochastics;
		}
				
		for (i=0;i<n;i++) {//take every point and if it is not satisfied, get its gradient
			minr=-1;
			if (Wanna[i]!=Current[i]) {
				for (j=0;j<ms;j++) {
					for (k=0;k<SteinerNeighborsMax[j];k++) {
						if ( (SteinerNeighbors_L[j][k]==Wanna[i]) || (SteinerNeighbors_R[j][k]==Wanna[i]) ) {
							a=j;
							b=SteinerNeighbors[j][k];
							//[xa+t(xb-xa)-Ob_x]2+[ya+t(yb-ya)-Ob_y]2 -> min
							//tt[(xb-ax)2+(yb-ya)2] + t[(yb-ya)(ax-Ob_x)+(xb-xa)(ya-Ob_y)] -> min
							//2t[(xb-ax)2+(yb-ya)2] + [(yb-ya)(ax-Ob_x)+(xb-xa)(ya-Ob_y)] = 0
							dx=Steiner_x[b]-Steiner_x[a];
							dy=Steiner_y[b]-Steiner_y[a];
							t=dx*dx+dy*dy;
							if (t>0) {
								t = - (dy*(Steiner_x[a]-Ob_x[i])+dx*(Steiner_y[a]-Ob_y[i])) / 2.0 / (dx*dx+dy*dy);
							} else {
								t=-1;
							}
							if ((t>=0)&&(t<=1)) {
								dx=Steiner_x[a]+dx*t-Ob_x[i];
								dy=Steiner_y[a]+dy*t-Ob_y[i];
								r=dx*dx+dy*dy;
								if ((r>0)&&((minr<0)||(r<minr))) {
									gx=-dx*f1(r)/r;
									gy=-dy*f1(r)/r;
									
									recall_a=a;
									recall_b=b;
									minr=r;
									mass_a=1-t;
									mass_b=t;
									
									if (Steiner_moveable[a]==false) {
										mass_a=0;
										mass_b=1;
									}
									
									if (Steiner_moveable[b]==false) {
										mass_a=1;
										mass_b=0;
									}
								}
							} else {//seems we have to choose between t={0,1}
								//try t=0
								dx=Steiner_x[a]-Ob_x[i];
								dy=Steiner_y[a]-Ob_y[i];
								r=dx*dx+dy*dy;
								if ((r>0)&&((minr<0)||(r<minr))&&(Steiner_moveable[a]==true)) {
									gx=-dx*f1(r)/r;
									gy=-dy*f1(r)/r;
									
									recall_a=a;
									recall_b=b;
									minr=r;
									mass_a=1;//full gradient blast goes on a, t=0
									mass_b=0;
								}
								
								//try t=1
								dx=Steiner_x[b]-Ob_x[i];
								dy=Steiner_y[b]-Ob_y[i];
								r=dx*dx+dy*dy;
								if ((r>0)&&((minr<0)||(r<minr))&&(Steiner_moveable[b]==true)) {
									gx=-dx*f1(r)/r;
									gy=-dy*f1(r)/r;
									
									recall_a=a;
									recall_b=b;
									minr=r;
									mass_a=0;//full gradient blast goes on b, t=1
									mass_b=1;
								}
							}
						}
					}
				}
				if (minr>0) {
					//now we know who is in charge and we need to give him a gradient
					grad_x[recall_a]+=gx*mass_a;
					grad_y[recall_a]+=gy*mass_a;
					grad_x[recall_b]+=gx*mass_b;
					grad_y[recall_b]+=gy*mass_b;
					
					Steiner_SAT+=sqrt(minr);
				}
			}
		}
		
		//now time to apply gradient descent finally
		for (i=0;i<ms;i++) {
			r=grad_x[i]*grad_x[i]+grad_y[i]*grad_y[i];
			
			if ((r>0)&&(Steiner_moveable[i]==true)) {
				r=sqrt(r);
				step=initial_speed;
				
				//trying to make a step
				xa=Steiner_x[i];
				ya=Steiner_y[i];
				xb=Steiner_x[i]+grad_x[i]/r*step;
				yb=Steiner_y[i]+grad_y[i]/r*step;
				
				iter=iter_base;
				stilldo=true;
				
				while (iter>=0) {//try several times or till it becomes feasible
					grad_x[i]=0;
					grad_y[i]=0;
					
					stilldo=false;
					
					//check the feasibility and project the gradient if requires
					for (int j=0;j<SteinerConstraints_max[i];j++) {
						a=SteinerConstraints_A[i][j];
						b=SteinerConstraints_B[i][j];
						
						dx=Steiner_x[b]-Steiner_x[a];
						dy=Steiner_y[b]-Steiner_y[a];
						gx=xb-Steiner_x[a];
						gy=yb-Steiner_y[a];
						
						//<[-dy,dx];[gx,gy]> >= 0 is a constraint
						if ( -dy*gx+dx*gy < 0 ) {//correct it if unfeasible, try to correct the gradient
							stilldo=true;//seems we are not done yet
							//<[-dy,dx];[gx-t*dy,gy+t*dx]> = 0 let's try to find
							//dy[gx-t*dy]=dx[gy+t*dx] let's try to find
							//dy*gx-t*dy*dy=dx*gy+t*dx*dx
							//dy*gx-dx*gy=t*[dx*dx+dy*dy]
							
							t=dx*dx+dy*dy;
							
							if (t>0) {
								t=(dy*gx-dx*gy)/t;
								grad_x[i]-=dy*t;
								grad_y[i]+=dx*t;
							}
						}
					}
					
					//check that all the neighbors make an angle <=180
					for (int j=0;j<SteinerNeighborsMax[i];j++) {
						k=SteinerNeighbors[i][j];
						dx=yb-Steiner_y[k];
						dy=Steiner_x[k]-xb;
						
						if (j+1==SteinerNeighborsMax[i]) {
							k=SteinerNeighbors[i][0];//compare with first to complete the circle
						} else {
							k=SteinerNeighbors[i][j+1];
						}
						gx=Steiner_x[k]-xb;
						gy=Steiner_y[k]-yb;
						
						//<[gx,gy];[dx,dy]> >= 0 is a constraint
						if ( gx*dx+gy*dy < 0 ) {
							stilldo=true;//seems we are not done yet
							vx=Steiner_y[SteinerNeighbors[i][j]]-Steiner_y[SteinerNeighbors[i][j+1]];
							vy=Steiner_x[SteinerNeighbors[i][j+1]]-Steiner_x[SteinerNeighbors[i][j]];
							
							//<[gx-vx*t,gy-vy*t];[dx,dy]> = 0 let's try to find
							//dx*gx-vx*dx*t+dy*gy-vy*dy*t = 0
							//dx*gx+dy*gy = [dy*vy+dx*vx]t
							
							t=dy*vy+dx*vx;
							
							if (t>0) {
								t=(dx*gx+dy*gy)/t;
								grad_x[i]+=vx*t;
								grad_y[i]+=vy*t;
							}
						}
					}
					
					
					if (base_stays_on_cluster==true) {//the base point has to be inside its cluster (new constraint)
						for (int j=0;j<SteinerNeighborsMax[i];j++) {
							a=i;
							b=SteinerNeighbors[i][j];
							
							//check the left cluster
							k=SteinerNeighbors_L[i][j];
							gx=Base_x[k]-Steiner_x[b];
							gy=Base_y[k]-Steiner_y[b];
							dx=yb-Steiner_y[b];
							dy=Steiner_x[b]-xb;
							vx=gy;//choosen vectors without xb,yb to increase stability of the projection
							vy=-gx;
							if (gx*dx+gy*dy<0) {
								stilldo=true;//seems we are not done yet
								//<[gx,gy];[dx+t*vy,dy-t*vx> = 0
								//gx[dx+t*vy]+gy[dy-t*vx] = 0
								//gxdx+gydy = tgyvx-tgxvy
								//[gx*dx+gy*dy] = t[gy*vx-gx*vy]
								t=(gx*dx+gy*dy)/(gy*vx-gx*vy);
								grad_x[i]+=t*vx;
								grad_y[i]+=t*vy;
							}
							
							//check the right cluster
							k=SteinerNeighbors_R[i][j];
							gx=Base_x[k]-Steiner_x[b];
							gy=Base_y[k]-Steiner_y[b];
							dx=Steiner_y[b]-yb;
							dy=xb-Steiner_x[b];
							vx=gy;//choosen vectors without xb,yb to increase stability of the projection
							vy=-gx;
							if (gx*dx+gy*dy<0) {
								stilldo=true;//seems we are not done yet
								//<[gx,gy];[dx-t*vy,dy+t*vx> = 0
								//gx[dx-t*vy]+gy[dy+t*vx] = 0
								//gxdx+gydy = -tgyvx+tgxvy
								//[gx*dx+gy*dy] = t[-gy*vx+gx*vy]
								t=(gx*dx+gy*dy)/(gx*vy-gy*vx);
								grad_x[i]+=t*vx;
								grad_y[i]+=t*vy;
							}
						}
					}
					
					xb+=grad_x[i];
					yb+=grad_y[i];
					
					if (stilldo==false) {iter=0;}
					iter--;
				}
				
				if (stilldo==false) {
					
					//remember, we need to update some vertices
					//apply QTree for it, our triangles are [xa,ya]-[xb,yb]-[Steiner_Neighbors_i_{j}] where j runs through all the neighbors of i
					int run;
					bool condition_big[M_],condition_small[M_],condition_fixed[M_];
					run=0;
					
					for (int i=0;i<QTree_max;i++) {
						condition_fixed[i]=false;
					}
					
					while (run!=-1) {
						//firstly suppose that there is no reason to go anywhere deeper
						condition_big[run]=false;
						condition_small[run]=false;
						
						if (condition_fixed[run]==false) {
							//only if we have not touched it yet, otherwise wait till it gets observed completley and do not touch its condition, it has been touched yet
							condition_fixed[run]=true;
							
							//prepare to go deeper if required
							if (QTree_variable[run]=='x') {
								if (xa>=QTree_treshold[run]) {condition_big[run]=true;}
								if (xb>=QTree_treshold[run]) {condition_big[run]=true;}
								for (int j=0;(j<SteinerNeighborsMax[i])&&(condition_big[run]==false);j++) {
									if (Steiner_x[SteinerNeighbors[i][j]]>=QTree_treshold[run]) {condition_big[run]=true;}
								}
								
								if (xa<QTree_treshold[run]) {condition_small[run]=true;}
								if (xb<QTree_treshold[run]) {condition_small[run]=true;}
								for (int j=0;(j<SteinerNeighborsMax[i])&&(condition_small[run]==false);j++) {
									if (Steiner_x[SteinerNeighbors[i][j]]<QTree_treshold[run]) {condition_small[run]=true;}
								}
								
								if (increased_accuracy==true) {
									if (xa==QTree_treshold[run]) {condition_small[run]=true;}
									if (xb==QTree_treshold[run]) {condition_small[run]=true;}
									for (int j=0;(j<SteinerNeighborsMax[i])&&(condition_small[run]==false);j++) {
										if (Steiner_x[SteinerNeighbors[i][j]]==QTree_treshold[run]) {condition_small[run]=true;}
									}
								}
							}
							if (QTree_variable[run]=='y') {
								if (ya>=QTree_treshold[run]) {condition_big[run]=true;}
								if (yb>=QTree_treshold[run]) {condition_big[run]=true;}
								for (int j=0;(j<SteinerNeighborsMax[i])&&(condition_big[run]==false);j++) {
									if (Steiner_y[SteinerNeighbors[i][j]]>=QTree_treshold[run]) {condition_big[run]=true;}
								}
								
								if (ya<QTree_treshold[run]) {condition_small[run]=true;}
								if (yb<QTree_treshold[run]) {condition_small[run]=true;}
								for (int j=0;(j<SteinerNeighborsMax[i])&&(condition_small[run]==false);j++) {
									if (Steiner_y[SteinerNeighbors[i][j]]<QTree_treshold[run]) {condition_small[run]=true;}
								}
								
								if (increased_accuracy==true) {
									if (ya==QTree_treshold[run]) {condition_small[run]=true;}
									if (yb==QTree_treshold[run]) {condition_small[run]=true;}
									for (int j=0;(j<SteinerNeighborsMax[i])&&(condition_small[run]==false);j++) {
										if (Steiner_y[SteinerNeighbors[i][j]]==QTree_treshold[run]) {condition_small[run]=true;}
									}
								}
							}
							
							//if we are on the leaf, highlight the whole leaf
							if (QTree_variable[run]=='l') {
								j=QTree_Storage[run];
								for (int k=0;k<Storage_max[j];k++) {
									doupdate[Storage[j][k]]=true;
								}
							}
						}
						
						//now go deeper if required
						if (condition_small[run]==true) {
							condition_small[run]=false;//when we will rise up back, will try consitions_big yet
							run=QTree_small[run];
						} else {
							if (condition_big[run]==true) {
								run=QTree_big[run];
							} else {//seems we have observed the whole subtree, go level higher back
								run=QTree_back[run];
							}
						}
					}
					
					//and finally update the Steiner vertice
					Steiner_x[i]=xb;
					Steiner_y[i]=yb;
				}
			}
		}
		
		//and now update all the vertices who may have been touched
		for (int i=0;i<N;i++) {
			if (doupdate[i]==true) {
				j=-2;
				while (j==-2) {j=determine_class(Ob_x[i],Ob_y[i]);}
				Current[i]=j;
			}
		}
			
	}
}

bool skills_satisfy(bool A[Tmax],bool B[Tmax]) {
	for (int i=0;i<Tmax;i++) {
		if ((A[i]==true)&&(B[i]==false)) {return false;}
	}
	return true;
}

void routes_greedy(float penalty) {//generates the greedy routes
	double r,minr;
	int lead_id,lead_m,lead_freed,nodes_has[M];
	for (int i=0;i<n;i++) {
		Ob_freed[i]=i;
		
		Ob_forward[i]=-1;
		Ob_back[i]=-1;
	}
	Ob_freed_max=n;
	
	int w=0;
	
	for (int i=0;i<m-4;i++) {//prepare for generating the routes
		Base_max[i]=1;
		
		Base_start[i]=-1;
		Base_finish[i]=-1;
		nodes_has[i]=0;
		
		Base_Worker[i][0]=w;
		w++;
	}
	
	bool cando,havewe;
	
	minr=-initial_summ;
	while ((Ob_freed_max>0)&&(minr>=-initial_summ)&&(is_time_still()==true)) {//for every base add such routes as requires
		minr=-initial_summ*2;
		for (int i=0;i<m-4;i++) {//guessing the vertice to connect
			for (int k=0;k<Ob_freed_max;k++) {
				if ((Current[Ob_freed[k]]==i)||(penalty>=0)) {
					cando=true;
					r=Ob_Base_dist[Ob_freed[k]][i]-Ob_weight[Ob_freed[k]];
					
					if ((Current[Ob_freed[k]]!=i)&&(penalty>=0)) {r+=penalty;}
					
					havewe=false;
					for (int j=0;j<Base_max[i];j++) {
						if (skills_satisfy(Ob_skill[Ob_freed[k]],Worker_skill[Base_Worker[i][j]])==true) {
							havewe=true;
						}
					}
					if (havewe==false) {
						if (penalty>=0) {r+=penalty;}
						if (penalty<0) {cando=false;}
					}
					
					if (((Nodes_max>0)&&(nodes_has[i]+1>Nodes_max))||(nodes_has[i]+1>Base_nodes_max[i])) {
						if (penalty>=0) {r+=penalty;}
						if (penalty<0) {cando=false;}
					}
					
					if (Base_finish[i]==-1) {
						r+=Base_weight[i];
					}
					
					if (((minr<-initial_summ)||(r<minr))&&(cando==true)) {
						minr=r;
						lead_id=Ob_freed[k];
						lead_m=i;
						lead_freed=k;
					}
				}
			}
		}
		if (minr>=-initial_summ) {
			nodes_has[lead_m]+=1;
			if (Base_finish[lead_m]==-1) {
				Base_start[lead_m]=lead_id;
				Base_finish[lead_m]=lead_id;
			} else {
				Ob_forward[Base_finish[lead_m]]=lead_id;
				Ob_back[lead_id]=Base_finish[lead_m];
				Base_finish[lead_m]=lead_id;
			}
			
			Ob_freed[lead_freed]=Ob_freed[Ob_freed_max-1];
			Ob_freed_max--;
		}
	}
}

void routes_greedy_fast(float penalty, float rad_x, float rad_y) {//generates the greedy routes
	double r,minr;
	int lead_id,lead_m,lead_link,nodes_has[M];
	int QTree_elements[N];
	int QTree_cond[N];
	bool isfreed[N];
	
	int run;
	
	//preprocessing the QTree for our purposes
	
	for (int i=0;i<n;i++) {isfreed[i]=true;}
	
	for (int i=0;i<QTree_max;i++) {QTree_cond[i]=0;}
	
	run=0;
	while ((QTree_cond[0]!=2)||(run>0)) {
		if (QTree_variable[run]=='l') {//leave, finaly
			QTree_elements[run]=Storage_max[QTree_Storage[run]];
			run=QTree_back[run];
		} else {
			if (QTree_cond[run]==0) {//<
				QTree_cond[run]=1;
				run=QTree_small[run];
			} else {
				if (QTree_cond[run]==1) {//>=
					QTree_cond[run]=2;
					run=QTree_big[run];
				} else {//done
					QTree_elements[run]=QTree_elements[QTree_big[run]]+QTree_elements[QTree_small[run]];
					run=QTree_back[run];
				}
			}
		}
	}//now run==0
	QTree_elements[run]=QTree_elements[QTree_big[run]]+QTree_elements[QTree_small[run]];
	
	//we seems to be ready to go fast now
	
	
	
	for (int i=0;i<n;i++) {
		Ob_forward[i]=-1;
		Ob_back[i]=-1;
	}
	Ob_freed_max=n;
	
	int w=0;
	
	for (int i=0;i<m-4;i++) {//prepare for generating the routes
		Base_max[i]=1;
		
		Base_start[i]=-1;
		Base_finish[i]=-1;
		nodes_has[i]=0;
		
		Base_Worker[i][0]=w;
		w++;
	}
	
	bool cando,havewe;
	bool get_all_now=false;//stage 1 - fast, stage 2 - collect all the garbage which is still left and get it
	int id;
	
	minr=-initial_summ;
	while ((QTree_elements[0]>0)&&(minr>=-initial_summ)&&(is_time_still()==true)) {//for every base add such routes as requires
		minr=-initial_summ*2;
		
		for (int i=0;i<m-4;i++) {//guessing the vertice to connect
			
			for (int i=0;i<QTree_max;i++) {QTree_cond[i]=0;}
			
			if (get_all_now==true) {
				
				for (id=0;id<n;id++) {
					
					if (isfreed[id]==true) {
						
						//try to connect id exactly
						if ((Current[id]==i)||(penalty>=0)) {
							cando=true;
							r=Ob_Base_dist[id][i]-Ob_weight[id];
							
							if ((Current[id]!=i)&&(penalty>=0)) {r+=penalty;}
							
							havewe=false;
							for (int j=0;j<Base_max[i];j++) {
								if (skills_satisfy(Ob_skill[id],Worker_skill[Base_Worker[i][j]])==true) {
									havewe=true;
								}
							}
							if (havewe==false) {
								if (penalty>=0) {r+=penalty;}
								if (penalty<0) {cando=false;}
							}
							
							if (((Nodes_max>0)&&(nodes_has[i]+1>Nodes_max))||(nodes_has[i]+1>Base_nodes_max[i])) {
								if (penalty>=0) {r+=penalty;}
								if (penalty<0) {cando=false;}
							}
							
							if (Base_finish[i]==-1) {
								r+=Base_weight[i];
							}
							
							if (((minr<-initial_summ)||(r<minr))&&(cando==true)) {
								minr=r;
								lead_m=i;
								lead_id=id;
								lead_link=run;
							}
						}
						
					}
					
				}
				
			} else {
				run=0;//searching for the objects close enough for being checked
				while ((QTree_cond[0]!=2)||(run>0)) {
					
					if (QTree_variable[run]=='l') {//leave, finaly
						//trying to connect it whole
						
						for (int k=0;k<Storage_max[QTree_Storage[run]];k++) {
							
							id=Storage[QTree_Storage[run]][k];
							
							if (isfreed[id]==true) {
								
								//try to connect id exactly
								if ((Current[id]==i)||(penalty>=0)) {
									cando=true;
									r=Ob_Base_dist[id][i]-Ob_weight[id];
									
									if ((Current[id]!=i)&&(penalty>=0)) {r+=penalty;}
									
									havewe=false;
									for (int j=0;j<Base_max[i];j++) {
										if (skills_satisfy(Ob_skill[id],Worker_skill[Base_Worker[i][j]])==true) {
											havewe=true;
										}
									}
									if (havewe==false) {
										if (penalty>=0) {r+=penalty;}
										if (penalty<0) {cando=false;}
									}
									
									if (((Nodes_max>0)&&(nodes_has[i]+1>Nodes_max))||(nodes_has[i]+1>Base_nodes_max[i])) {
										if (penalty>=0) {r+=penalty;}
										if (penalty<0) {cando=false;}
									}
									
									if (((minr<-initial_summ)||(r<minr))&&(cando==true)) {
										minr=r;
										lead_m=i;
										lead_id=id;
										lead_link=run;
									}
								}
								
							}
							
						}
						
						run=QTree_back[run];
						
					} else {
						if (QTree_cond[run]==0) {//<
							
							if (QTree_elements[QTree_small[run]]==0) {//do not have anything on there yet
								QTree_cond[run]=1;
							} else {
								if (QTree_variable[run]=='x') {
									if ((Base_x[i]<QTree_treshold[run]+rad_x)||(QTree_elements[QTree_big[run]]==0)) {//let's observe it, may be useful
										QTree_cond[run]=1;
										run=QTree_small[run];
									} else {//No,let's try another
										QTree_cond[run]=1;
									}
								} else {
									if (QTree_variable[run]=='y') {
										if ((Base_y[i]<QTree_treshold[run]+rad_y)||(QTree_elements[QTree_big[run]]==0)) {//let's observe it, may be useful
											QTree_cond[run]=1;
											run=QTree_small[run];
										} else {//No,let's try another
											QTree_cond[run]=1;
										}
									}
								}
							}
							
						} else {
							if (QTree_cond[run]==1) {//>=
								
								if (QTree_elements[QTree_big[run]]==0) {//do not have anything on there yet
									QTree_cond[run]=2;
								} else {
									if (QTree_variable[run]=='x') {
										if ((Base_x[i]>=QTree_treshold[run]-rad_x)||(QTree_elements[QTree_small[run]]==0)) {//let's observe it, may be useful
											QTree_cond[run]=2;
											run=QTree_big[run];
										} else {//No,let's try another
											QTree_cond[run]=2;
										}
									} else {
										if (QTree_variable[run]=='y') {
											if ((Base_y[i]>=QTree_treshold[run]-rad_y)||(QTree_elements[QTree_small[run]]==0)) {//let's observe it, may be useful
												QTree_cond[run]=2;
												run=QTree_big[run];
											} else {//No,let's try another
												QTree_cond[run]=2;
											}
										}
									}
								}
								
							} else {//done
								run=QTree_back[run];
							}
						}
					}
				}
				
			}
		}
		
		if (minr>=-initial_summ) {
			nodes_has[lead_m]+=1;
			
			if (Base_finish[lead_m]==-1) {
				Base_start[lead_m]=lead_id;
				Base_finish[lead_m]=lead_id;
			} else {
				Ob_forward[Base_finish[lead_m]]=lead_id;
				Ob_back[lead_id]=Base_finish[lead_m];
				Base_finish[lead_m]=lead_id;
			}
			
			Ob_freed_max--;
			
			isfreed[lead_id]=false;
			
			run=lead_link;
			while (run!=0) {
				QTree_elements[run]--;
				run=QTree_back[run];
			}
			QTree_elements[run]--;
		}
		
		if ((minr<-initial_summ)&&(get_all_now==false)) {//stage 2 - where we add all the rest without thinking of any optimization, just get all who is still left
			minr=0;//it is very important, because sometimes this optimization may give some vertices unvisited on the fast stage 1
			get_all_now=true;
		}
	}
}

void route_break(int i,int power) {
	int g_,g,l=0,a,b;
	
	g=Base_start[i];
	while ((g!=Base_finish[i])&&(g!=-1)) {
		g=Ob_forward[g];
		l++;
	}
	
	a=rand_huge()%(l+1);
	b=a;
	while ((power>0)&&((a>0)||(b<l))) {
		if ((a>0)&&(b<l)) {
			if (rand()%2==0) {
				a--;
			} else {
				b++;
			}
		} else {
			if (b<l) {
				b++;
			}
			if (a>0) {
				a--;
			}
		}
		power--;
	}
	
	int begin=-1,end=-1;
	
	l=0;
	g=Base_start[i];
	while ((g!=Base_finish[i])&&(g!=-1)) {
		g_=Ob_forward[g];
		
		if (l==a-1) {begin=g;}
		if (l==b+1) {end=g;}
		
		if ((l>=a)&&(l<=b)) {
			//if it is cutten, write it as freed
			Ob_freed[Ob_freed_max]=g;
			Ob_freed_max++;
		}
		
		if ((l>=a-1)&&(l<=b)) {Ob_forward[g]=-1;}
		if ((l>=a)&&(l<=b+1)) {Ob_back[g]=-1;}
		
		g=g_;
		l++;
	}
	
	if (l==a-1) {begin=g;}
	if (l==b+1) {end=g;}
	
	if ((l>=a-1)&&(l<=b)) {Ob_forward[g]=-1;}
	if ((l>=a)&&(l<=b+1)) {Ob_back[g]=-1;}
	
	if ((l>=a)&&(l<=b)) {
		//if it is cutten, write it as freed
		Ob_freed[Ob_freed_max]=g;
		Ob_freed_max++;
	}
	
	Ob_forward[begin]=end;
	Ob_back[end]=begin;
	
	Base_nodes[i]-=b-a+1;
	
	if (a==0) {Base_start[i]=-1;}
	if (b>=l) {Base_finish[i]=-1;}
}

void routes_repair(float penalty,float T) {//T is being variated from 0 up to infinity, it is a coldness
	double r,minr;
	int lead_id,lead_m,lead_freed,g,g_;
	int add_nodes[M];
	
	bool canadd,havewe;
	
	for (int i=0;i<m-4;i++) {//prepare for generating the routes
		add_nodes[i]=0;
	}
	
	minr=-initial_summ;
	while ((Ob_freed_max>0)&&(minr>=-initial_summ)) {//for every base check its feasibility and if it is not true, rebuild
		minr=-initial_summ*2;
		for (int i=0;i<m-4;i++) {//prepare for generating the routes
			for (int k=0;k<Ob_freed_max;k++) {
				
				if ((Current[Ob_freed[k]]==i)||(penalty>=0)) {
					canadd=true;
					r=Ob_Base_dist[Ob_freed[k]][i]-Ob_weight[Ob_freed[k]];
					if (Current[Ob_freed[k]]!=i) {r+=penalty;}
					
					havewe=false;
					for (int j=0;j<Base_max[i];j++) {
						if (skills_satisfy(Ob_skill[Ob_freed[k]],Worker_skill[Base_Worker[i][j]])==true) {
							havewe=true;
						}
					}
					if (havewe==false) {
						if (penalty>=0) {r+=penalty;}
						if (penalty<0) {canadd=false;}
					}
					
					if ((add_nodes[i]+Base_nodes[i]+1>Nodes_max)||(add_nodes[i]+Base_nodes[i]+1>Base_nodes_max[i])) {
						if (penalty>=0) {r+=penalty;}
						if (penalty<0) {canadd=false;}
					}
					
					if (Base_finish[i]==-1) {
						r+=Base_weight[i];
					}
					
					if (
						(
							(minr<-initial_summ)||(r<minr)||
							((r>minr)&&(rand_huge()%10000/10000.0<exp((minr-r)*T)))
						)&&(
						canadd==true
						)
					) {
						minr=r;
						lead_id=Ob_freed[k];
						lead_m=i;
						lead_freed=k;
					}
				}
				
			}
		}
		if (minr>=-initial_summ) {
			add_nodes[lead_m]++;
			
			if (Base_finish[lead_m]==-1) {
				Base_start[lead_m]=lead_id;
				Base_finish[lead_m]=lead_id;
			} else {
				Ob_forward[Base_finish[lead_m]]=lead_id;
				Ob_back[lead_id]=Base_finish[lead_m];
				Base_finish[lead_m]=lead_id;
			}
			
			Ob_freed[lead_freed]=Ob_freed[Ob_freed_max-1];
			Ob_freed_max--;
		}
	}
}

void routes_observe_empty() {
	int g;
	bool feas[N];
	for (int i=0;i<n;i++) {feas[i]=0;}
	for (int i=0;i<m-4;i++) {
		
		g=Base_start[i];
		if (g!=-1) {
			feas[g]=1;
			while ((g!=Base_finish[i])&&(g!=-1)) {
				g=Ob_forward[g];
				if (g!=-1) {feas[g]=1;}
			}
		}
		
	}
	
	Ob_freed_max=0;
	
	for (int i=0;i<n;i++) {
		if (feas[i]==0) {
			Ob_forward[i]=-1;
			Ob_back[i]=-1;
			Ob_freed[Ob_freed_max]=i;
			Ob_freed_max++;
		}
	}
}

void skill_repair_prepare() {
	int g,T;
	
	for (int i=0;i<m-4;i++) {
		
		for (int k=0;k<Tmax;k++) {skill_wanna[i][k]=0;}
		T=0;
		
		g=Base_start[i];
		if (g!=-1) {
			
			while ((g!=Base_finish[i])&&(g!=-1)) {
				for (int k=1;k<Tmax;k++) {
					if (Ob_skill[g][k]==true) {
						skill_wanna[i][k]++;
						T++;
					}
				}
				
				g=Ob_forward[g];
			}
			
			if (g!=-1) {
				for (int k=1;k<Tmax;k++) {
					if (Ob_skill[g][k]==true) {
						skill_wanna[i][k]++;
						T++;
					}
				}
			}
			
		}
		
		if (T>0) {
			for (int k=0;k<Tmax;k++) {
				skill_wanna[i][k]/=(float)(T);
			}
		}
		
	}
}

void skill_repair(float coef) {
	float minr,r;//coef=1 -> jumping, coef=20 -> never change
	int id;
	int worker_prev[WO];
	for (int i=0;i<wo;i++) {
		worker_prev[i]=-1;
	}
	for (int i=0;i<m-4;i++) {
		for (int j=0;j<Base_max[i];j++) {
			worker_prev[Base_Worker[i][j]]=i;
		}
		Base_max[i]=0;
	}
	for (int i=0;i<wo;i++) {
		if (worker_prev[i]!=-1) {
			minr=0;
			id=worker_prev[i];
			for (int k=0;k<Tmax;k++) {
				if (Worker_skill[i][k]==true) {minr+=skill_wanna[id][k];}
			}
			minr*=coef;
		} else {
			minr=-1;
			id=-1;
		}
		for (int j=0;j<m-4;j++) {
			r=0;
			for (int k=0;k<Tmax;k++) {
				if (Worker_skill[i][k]==true) {r+=skill_wanna[j][k];}
			}
			if ((minr==-1)||(r>minr)) {
				r=minr;
				id=j;
			}
		}
		if (id!=-1) {
			Base_Worker[id][Base_max[id]]=i;
			Base_max[id]++;
		}
		//DBG("{%d->%d}",i,id);
	}
}

double evaluate_routes(float penalty) {
	int g,g_;
	int co;
	double ans,ans_,ans_pen;
	
	bool havewe;
	
	for (int i=0;i<m-4;i++) {
	
		ans_=0;
		ans_pen=0;
		
		g=Base_start[i];
		if (g==-1) {
			Base_nodes[i]=0;
			Base_skills[i]=0;
		} else {
			ans_+=Base_weight[i];
			
			Base_skills[i]=0;
			
			if ((Current[Base_start[i]]!=i)&&(penalty>=0)) {ans_pen+=penalty;}
			
			havewe=false;
			for (int j=0;j<Base_max[i];j++) {
				if (skills_satisfy(Ob_skill[Base_start[i]],Worker_skill[Base_Worker[i][j]])==true) {
					havewe=true;
				}
			}
			if (havewe==false) {
				Base_skills[i]++;
				ans_pen+=penalty;
			}
			
			Base_nodes[i]=1;
			
			ans_+=Ob_Base_dist[Base_start[i]][i];
			ans_-=Ob_weight[Base_start[i]];
			g_=-1;
			while ((g!=Base_finish[i])&&(g!=-1)&&(g_!=g)) {
				g_=g;
				g=Ob_forward[g];
				if (g!=-1) {
					ans_+=Ob_Base_dist[g][i];
					ans_-=Ob_weight[g];
					if ((Current[g]!=i)&&(penalty>=0)) {ans_pen+=penalty;}
					
					havewe=false;
					for (int j=0;j<Base_max[i];j++) {
						if (skills_satisfy(Ob_skill[g],Worker_skill[Base_Worker[i][j]])==true) {
							havewe=true;
						}
					}
					if (havewe==false) {
						Base_skills[i]++;
						ans_pen+=penalty;
					}
				}
				Base_nodes[i]++;
			}
			if ((g!=Base_finish[i])||(g==-1)||(g==g_)) {return -1;}
			
			co=Base_nodes[i];
			
			if (penalty>=0) {
				if ((co<Nodes_min)&&(Nodes_min>=0)) {ans_pen+=penalty*(Nodes_min-co);}
				if ((co>Nodes_max)&&(Nodes_max>=0)) {ans_pen+=penalty*(co-Nodes_max);}
			}
			
			ans+=ans_+ans_pen;
		}
		
	}
	return ans+initial_summ;
}

int evaluate_unfeasibility() {
	int g,ans=0;
	for (int i=0;i<m-4;i++) {
		
		g=Base_start[i];
		if (g!=-1) {
			while ((g!=Base_finish[i])&&(g!=-1)) {
				if (Current[g]!=i) {ans++;}
				g=Ob_forward[g];
			}
			
			if ((g!=Base_finish[i])||(g==-1)) {
				return -1;
			}
			
			if (Current[Base_finish[i]]!=i) {ans++;}
		}
		
	}
	return ans;
}

int evaluate_unfeasibility_Nodes() {//requires running evaluate_routes() before
	int ans=0,co;
	int nod_min=-1,nod_max=-1;
	for (int i=0;i<m-4;i++) {
		co=Base_nodes[i];
		if (co>0) {
			// ТУТ ИСПРАВИЛ БАГ. Знаки были в обратную сторону
			if ((nod_min==-1)||(co<nod_min)) {nod_min=co;}
			if ((nod_max==-1)||(co>nod_max)) {nod_max=co;}
		}
		if (co>Base_nodes_max[i]) {
			ans++;
		}
	}
	if (nod_max-nod_min>gap_treshold) {
		
		int TT[M],temp;
		
		for (int i=0;i<m-4;i++) {
			TT[i]=Base_nodes[i];
		}
		
		for (int i=0;i<m-4;i++) {
			for (int j=0;j<m-5;j++) {
				if (TT[j]>TT[j+1]) {
					temp=TT[j];
					TT[j]=TT[j+1];
					TT[j+1]=temp;
				}
			}
		}
		
		temp=TT[(int)((m-4)/2)];
		
		for (int i=0;i<m-4;i++) {
			co=Base_nodes[i];
			if ((co<temp-gap_treshold/2+0.5) || (co>temp+gap_treshold/2-0.5)) {ans++;}
		}
		
		Nodes_max=temp+gap_treshold/2+0.5;
		Nodes_min=temp-gap_treshold/2-0.5;
	}
	return ans;
}

int evaluate_unfeasibility_Skills() {//requires running evaluate_routes() before
	int ans=0;
	for (int i=0;i<m-4;i++) {
		ans+=Base_skills[i];
	}
	return ans;
}

void routes_variate(int iter_huge,int iter,float penalty,float cooling,float maxcool,float alpha) {//block R
	double goal,goal_best;
	float Temp;
	
	char doing;
	bool didsth;
	
	float distribution[M];
	char distribution_type[M];//to understand who is guilty and how
	
	for (int i=0;i<m;i++) {
		Base_max[i]=1;
	}
	
	float a,b;
	int a_num,b_num;
	
	float penalty_;
	
	int num;
	
	Nodes_min=-1;
	Nodes_max=-1;
	
	//routes_greedy(penalty);
	routes_greedy_fast(penalty,global_rad_x,global_rad_y);
	routes_save(0);
	goal_best=evaluate_routes(penalty);
	
	char per_prev=0;
	
	for (int iitt=0;(iitt<iter_huge)&&(is_time_still()==true);iitt++) {
		
		if (penalty<0) {
			penalty_=penalty;
		} else {
			if (iitt<iter_huge/2) {
				penalty_=penalty*iitt/(iter_huge/2);
			} else {
				penalty_=penalty;
			}
		}
		
		if (iitt*10/iter_huge>per_prev) {
			per_prev++;
			DBG("[=]");
		}
		
		num=0;
		Temp=0;
		
		//routes_greedy(penalty_);
		routes_greedy_fast(penalty_,global_rad_x,global_rad_y);
		
		for (int i=0;i<m-4;i++) {
			distribution[i]=1.0/(m-4);
		}
		
		for (int tt=0;(tt<iter)&&(is_time_still()==true);tt++) {
			Temp+=cooling;
			if (Temp>maxcool) {Temp=0;}
			
			skill_repair_prepare();
			skill_repair(Temp+1.0);
			for (int i=0;i<m-4;i++) {
				if ((rand_huge()%10000)/10000.0<distribution[i]) {
					num=0;
					while ((rand()%10000/10000.0<coeffs[1])&&(num<n)) {
						num++;
					}
					route_break(i,2+num);
				}
			}
			routes_observe_empty();
			routes_repair(penalty_,sqrt(Temp/cooling)*cooling);
			goal=evaluate_routes(penalty_);
			if ((goal>=0)&&((goal<goal_best)||(goal_best<0))) {
				goal_best=goal;
				routes_save(0);
			}
			
			a=0;
			a_num=0;
			b=0;
			b_num=0;
			for (int i=0;i<m-4;i++) {
				distribution_type[i]=0;//did nothing bad
				if ((Base_nodes[i]>Nodes_max)&&(Nodes_max>=0)) {
					distribution_type[i]=3;
				}
				if (Base_nodes[i]<Nodes_min) {
					distribution_type[i]=4;
				}
				if (Base_skills[i]>0) {
					distribution_type[i]=5;
				}
				if (distribution_type[i]==0) {
					a+=distribution[i];
					a_num++;
				} else {
					b+=distribution[i];
					b_num++;
				}
			}
			if ((a_num>0)&&(b_num>0)) {
				a/=a_num;
				b/=b_num;
				for (int i=0;i<m-4;i++) {
					if (distribution_type[i]==0) {
						distribution[i]=distribution[i]*(1-alpha)-alpha*b;
					} else {
						distribution[i]=distribution[i]*(1-alpha)+alpha*a;
					}
				}
			}
			routes_load(0);
			Ob_freed_max=0;
			evaluate_routes(penalty_);
		}
		
	}
	routes_load(0);
	Ob_freed_max=0;
}

void geometry_variate(int itermax, int cycle, int iter_each, int iter_each_correct) {//block G
	int counting_bad,counting_bad_min=-1,iter=0;
	float SAT_min=-1;
	
	Steiner_load(0);
	
	determine_class_prepare();
	determine_class_forall();
	
	char per_prev=0;
	
	for (int i=0;(i<itermax)&&(is_time_still()==true);i++) {
		if (i*10/itermax>per_prev) {
			per_prev++;
			DBG("[=]");
		}
		if (i%cycle==0) {
			iter=0;
			if (i%(cycle*2)==0) {
				Steiner_load(0);
			} else {
				Steiner_load(1);
			}
			determine_class_prepare();
			determine_class_forall();
		}
		
		/*determine_class_prepare();
		determine_class_forall();//replace further, it is way too slow*/
		
		Steiner_variate(coeffs[2]/sqrt(iter+1),coeffs[3]/(iter+1),iter_each,iter_each_correct);
		
		/*determine_class_prepare();
		determine_class_forall();//replace further, it is way too slow*/
		
		counting_bad=0;
		for (int j=0;j<n;j++) {
			if (Wanna[j]!=Current[j]) {counting_bad++;}
		}
		
		if ((counting_bad_min<0)||(counting_bad<counting_bad_min)||(SAT_min<0)||(Steiner_SAT<SAT_min)) {
			if ((counting_bad_min<0)||(counting_bad<counting_bad_min)) {
				counting_bad_min=counting_bad;
				Steiner_save(0);
			}
			if ((SAT_min<0)||(Steiner_SAT<SAT_min)) {
				SAT_min=Steiner_SAT;
				Steiner_save(1);
			}
		}
		
	}
	
	Steiner_load(0);//loading the best found solution so far
	determine_class_prepare();
	determine_class_forall();
}

void routes_color() {//show G R's oppinion
	int g;
	for (int i=0;i<m-4;i++) {
		g=Base_start[i];
		while ((g!=Base_finish[i])&&(g!=-1)) {
			Wanna[g]=i;
			g=Ob_forward[g];
		}
		Wanna[Base_finish[i]]=i;
	}
}

void sample_generate(int nn,int mm,int ww,int wx,int wy,int stepaway,float prob_days,float prob_days_) {
	n=nn;
	m=mm;
	wo=ww;
	float xmin=0,xmax=0,ymin=0,ymax=0,dx,dy;
	for (int i=0;i<m;i++) {
		Base_weight[i]=rand()%10+10;
		Base_nodes_max[i]=n/(m-3)*2;
		Base_x[i]=wx*(rand_huge()%(100000)/100000.0)-wx/2.0;
		Base_y[i]=wy*(rand_huge()%(100000)/100000.0)-wy/2.0;
		if (Base_x[i]>xmax) {xmax=Base_x[i];}
		if (Base_x[i]<xmin) {xmin=Base_x[i];}
		if (Base_y[i]>ymax) {ymax=Base_y[i];}
		if (Base_y[i]<ymin) {ymin=Base_y[i];}
	}
	Base_x[m+0]=xmin-(xmax-xmin)*stepaway/2;
	Base_y[m+0]=ymin-(ymax-ymin)*stepaway/2;
	
	Base_x[m+1]=xmax+(xmax-xmin)*stepaway/2;
	Base_y[m+1]=ymin-(ymax-ymin)*stepaway/2;
	
	Base_x[m+2]=xmin-(xmax-xmin)*stepaway/2;
	Base_y[m+2]=ymax+(ymax-ymin)*stepaway/2;
	
	Base_x[m+3]=xmax+(xmax-xmin)*stepaway/2;
	Base_y[m+3]=ymax+(ymax-ymin)*stepaway/2;
	m+=4;
	
	global_rad_x=(xmax-xmin)/sqrt(n);
	global_rad_y=(ymax-ymin)/sqrt(n);
	
	initial_summ=0;
	
	for (int i=0;i<n;i++) {
		Ob_weight[i]=rand()%1000+10;
		initial_summ+=Ob_weight[i];
		Ob_x[i]=wx*(rand_huge()%(100000))/100000.0-wx/2.0;
		Ob_y[i]=wy*(rand_huge()%(100000))/100000.0-wy/2.0;
		
		for (int j=0;j<Tmax;j++) {
			Ob_skill[i][j]=false;
		}
		for (int j=0;j<Tmax;j++) {
			if (rand_huge()%100000/100000.0<prob_days) {Ob_skill[i][j]=true;}
		}
	}
	
	for (int i=0;i<n;i++) {
		for (int j=0;j<m;j++) {
			dx=Ob_x[i]-Base_x[j];
			dy=Ob_y[i]-Base_y[j];
			Ob_Base_dist[i][j]=sqrt(dx*dx+dy*dy);
		}
	}
	
	for (int i=0;i<wo;i++) {
		for (int j=0;j<Tmax;j++) {
			Worker_skill[i][j]=false;
		}
		for (int j=0;j<Tmax;j++) {
			if (rand_huge()%100000/100000.0<prob_days_) {Worker_skill[i][j]=true;}
		}
	}
}

void variate_coefficients() {
	float g;
	int j;
	for (int i=0;i<10;i++) {
		g=(rand_huge()%100000+1)/100000.0;
		j=0;
		while ((g>=0)&&(j<qu_coeffs)) {
			g+=coeffs_density[i][j];
			j++;
		}
		j--;
		if (j<0) {j=0;}
		coeffs[i]=(float)(coeffs_max[i]-coeffs_min[i])*j/qu_coeffs;
	}
}

void coeffs_feedback(float sh) {//<0 -> bad, 0 -> nothing, >0 -> good
	float g;
	int j;
	for (int i=0;i<10;i++) {
		g=0;
		j=(int)(coeffs[i]*qu_coeffs/(coeffs_max[i]-coeffs_min[i]));
		for (int k=0;k<qu_coeffs;k++) {
			if (k!=j) {
				g+=coeffs_density[i][k];
			}
		}
		g*=2.0/(1.0+exp(-sh))-1.0;
		for (int k=0;k<qu_coeffs;k++) {
			if (k!=j) {
				coeffs_density[i][k]-=g/(qu_coeffs-1);
			}
		}
		coeffs_density[i][j]+=g;
	}
}

// ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА
// ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА
// ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА
// ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА ЗОНА ПАРСИНГА

namespace {

std::string read_file(const char* path) {
	std::ifstream in(path, std::ios::binary);
	if (!in) {
		throw std::runtime_error("cannot open input");
	}
	return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void skip_ws(const std::string& s, std::size_t& i) {
	while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
		++i;
	}
}

struct Json {
	enum class Type { Null, Bool, Number, String, Array, Object };
	Type type = Type::Null;
	double number = 0;
	std::string str;
	std::vector<Json> array;
	std::vector<std::pair<std::string, Json>> object;

	static Json parse(const std::string& s, std::size_t& i);

	static Json parse_value(const std::string& s, std::size_t& i) {
		skip_ws(s, i);
		if (i >= s.size()) {
			throw std::runtime_error("unexpected end");
		}
		const char c = s[i];
		if (c == '{') {
			return parse_object(s, i);
		}
		if (c == '[') {
			return parse_array(s, i);
		}
		if (c == '"') {
			return parse_string(s, i);
		}
		if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
			return parse_number(s, i);
		}
		throw std::runtime_error("bad json");
	}

	static Json parse_object(const std::string& s, std::size_t& i) {
		Json out;
		out.type = Type::Object;
		++i;
		skip_ws(s, i);
		if (i < s.size() && s[i] == '}') {
			++i;
			return out;
		}
		while (true) {
			Json key = parse_string(s, i);
			skip_ws(s, i);
			if (i >= s.size() || s[i] != ':') {
				throw std::runtime_error("expected :");
			}
			++i;
			out.object.emplace_back(key.str, parse_value(s, i));
			skip_ws(s, i);
			if (i >= s.size()) {
				throw std::runtime_error("unclosed object");
			}
			if (s[i] == '}') {
				++i;
				return out;
			}
			if (s[i] != ',') {
				throw std::runtime_error("expected ,");
			}
			++i;
		}
	}

	static Json parse_array(const std::string& s, std::size_t& i) {
		Json out;
		out.type = Type::Array;
		++i;
		skip_ws(s, i);
		if (i < s.size() && s[i] == ']') {
			++i;
			return out;
		}
		while (true) {
			out.array.push_back(parse_value(s, i));
			skip_ws(s, i);
			if (i >= s.size()) {
				throw std::runtime_error("unclosed array");
			}
			if (s[i] == ']') {
				++i;
				return out;
			}
			if (s[i] != ',') {
				throw std::runtime_error("expected ,");
			}
			++i;
		}
	}

	static Json parse_string(const std::string& s, std::size_t& i) {
		Json out;
		out.type = Type::String;
		if (s[i] != '"') {
			throw std::runtime_error("expected string");
		}
		++i;
		while (i < s.size()) {
			const char c = s[i++];
			if (c == '"') {
				return out;
			}
			if (c == '\\' && i < s.size()) {
				out.str.push_back(s[i++]);
			} else {
				out.str.push_back(c);
			}
		}
		throw std::runtime_error("unterminated string");
	}

	static Json parse_number(const std::string& s, std::size_t& i) {
		Json out;
		out.type = Type::Number;
		const std::size_t start = i;
		if (s[i] == '-') {
			++i;
		}
		while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '.' || s[i] == 'e' ||
		                        s[i] == 'E' || s[i] == '+' || s[i] == '-')) {
			++i;
		}
		out.number = std::stod(s.substr(start, i - start));
		return out;
	}

	const Json* find(const std::string& key) const {
		if (type != Type::Object) {
			return nullptr;
		}
		for (const auto& entry : object) {
			if (entry.first == key) {
				return &entry.second;
			}
		}
		return nullptr;
	}
};

Json Json::parse(const std::string& s, std::size_t& i) {
	return parse_value(s, i);
}

std::vector<int> as_int_list(const Json& value) {
	std::vector<int> out;
	if (value.type != Json::Type::Array) {
		return out;
	}
	for (const Json& item : value.array) {
		out.push_back(static_cast<int>(item.number));
	}
	return out;
}

std::optional<std::vector<int>> cover_tasks(const std::vector<std::vector<int>>& workers,
                                           const std::vector<int>& site_tasks) {
	std::unordered_set<int> need(site_tasks.begin(), site_tasks.end());
	if (need.empty()) {
		return std::vector<int>{};
	}
	std::vector<int> chosen;
	std::unordered_set<int> used;
	while (!need.empty()) {
		int found = -1;
		for (int wid = 0; wid < static_cast<int>(workers.size()); ++wid) {
			if (used.count(wid) != 0) {
				continue;
			}
			bool covers_any = false;
			for (int task : need) {
				if (std::find(workers[wid].begin(), workers[wid].end(), task) != workers[wid].end()) {
					covers_any = true;
					break;
				}
			}
			if (covers_any) {
				found = wid;
				break;
			}
		}
		if (found < 0) {
			return std::nullopt;
		}
		chosen.push_back(found);
		used.insert(found);
		for (int skill : workers[found]) {
			need.erase(skill);
		}
	}
	std::sort(chosen.begin(), chosen.end());
	return chosen;
}

std::string minimal_solution(const Json& root) {
	const Json* sites = root.find("sites");
	const Json* offices = root.find("offices");
	const Json* workers_node = root.find("workers");
	if (sites == nullptr || offices == nullptr || workers_node == nullptr ||
	    sites->type != Json::Type::Array || offices->type != Json::Type::Array ||
	    workers_node->type != Json::Type::Array || sites->array.empty() || offices->array.empty()) {
		return "[]\n";
	}
	const Json& office0 = offices->array.front();
	const Json* max_size = office0.find("maxSize");
	if (max_size == nullptr || max_size->number < 1) {
		return "[]\n";
	}
	std::vector<std::vector<int>> workers;
	for (const Json& row : workers_node->array) {
		workers.push_back(as_int_list(row));
	}
	for (int site_id = 0; site_id < static_cast<int>(sites->array.size()); ++site_id) {
		const Json& site = sites->array[static_cast<std::size_t>(site_id)];
		const Json* tasks_node = site.find("tasks");
		std::vector<int> tasks = tasks_node != nullptr ? as_int_list(*tasks_node) : std::vector<int>{};
		const auto picked = cover_tasks(workers, tasks);
		if (!picked.has_value()) {
			continue;
		}
		std::ostringstream out;
		out << "[\n\t{\n\t\t\"officeId\": 0,\n\t\t\"sites\": [" << site_id << "],\n\t\t\"workers\": [";
		for (std::size_t j = 0; j < picked->size(); ++j) {
			if (j != 0) {
				out << ", ";
			}
			out << (*picked)[j];
		}
		out << "]\n\t}\n]\n";
		return out.str();
	}
	return "[]\n";
}

}  // namespace


void load_instance_from_json(const Json& root) {
    const Json* sites_json = root.find("sites");
    const Json* offices_json = root.find("offices");
    const Json* workers_json = root.find("workers");
    const Json* dist_json = root.find("distances");

    if (!sites_json || !offices_json || !workers_json || !dist_json) {
        throw std::runtime_error("Missing required JSON fields");
    }

    n = sites_json->array.size();
    m = offices_json->array.size(); // реальное количество
    wo = workers_json->array.size();

    // ПРОВЕРКИ ЛИМИТОВ на N=10000, M=500 - 4, WO=10000, Tmax=7
    
    if (n > N) {
        throw std::runtime_error("Too many sites: " + std::to_string(n) + " > " + std::to_string(N));
    }
    if (m > M - 4) {
        throw std::runtime_error("Too many offices: " + std::to_string(m) + " > " + std::to_string(M - 4));
    }
    if (wo > WO) {
        throw std::runtime_error("Too many workers: " + std::to_string(wo) + " > " + std::to_string(WO));
    }

    int max_skill_found = -1;
    int max_workers_on_office = 0;

    initial_summ = 0;
    float xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;

    for (int i = 0; i < n; ++i) {
        const auto& s = sites_json->array[i];
        Ob_x[i] = s.find("x")->number;
        Ob_y[i] = s.find("y")->number;
        Ob_weight[i] = s.find("reward")->number;
        initial_summ += Ob_weight[i];

        if (Ob_x[i] < xmin) xmin = Ob_x[i];
        if (Ob_x[i] > xmax) xmax = Ob_x[i];
        if (Ob_y[i] < ymin) ymin = Ob_y[i];
        if (Ob_y[i] > ymax) ymax = Ob_y[i];

        for (int j = 0; j < Tmax; j++) Ob_skill[i][j] = false;
        const Json* tasks = s.find("tasks");
        if (tasks && tasks->type == Json::Type::Array) {
            for (const auto& task : tasks->array) {
                int skill_id = (int)task.number;
                if (skill_id > max_skill_found) max_skill_found = skill_id;
                
                // ПРОВЕРКА ЛИМИТА Tmax
                if (skill_id >= Tmax || skill_id < 0) {
                    throw std::runtime_error("Skill ID out of bounds in site " + std::to_string(i));
                }
                Ob_skill[i][skill_id] = true;
            }
        }
    }

    for (int i = 0; i < m; ++i) {
        const auto& o = offices_json->array[i];
        Base_x[i] = o.find("x")->number;
        Base_y[i] = o.find("y")->number;
        Base_weight[i] = -o.find("cost")->number; 
        Base_nodes_max[i] = (int)o.find("maxSize")->number;
        
        if (Base_nodes_max[i] > max_workers_on_office) {
            max_workers_on_office = Base_nodes_max[i];
        }
    }

	for (int i = 0; i < wo; ++i) {
		for (int j = 0; j < Tmax; j++) Worker_skill[i][j] = false;

		const Json& skills = workers_json->array[i];
		if (skills.type == Json::Type::Array) {
			for (const auto& sk : skills.array) {
				int skill_id = (int)sk.number;
				if (skill_id > max_skill_found) max_skill_found = skill_id;

				// ПРОВЕРКА ЛИМИТА Tmax
				if (skill_id >= Tmax || skill_id < 0) {
					throw std::runtime_error(
						"Skill ID out of bounds in worker " + std::to_string(i));
				}
				Worker_skill[i][skill_id] = true;
			}
		}
	}
    
    // ПРОВЕРКА ЛИМИТА M__
    if (max_workers_on_office > M__) {
         throw std::runtime_error("maxSize in offices exceeds hardcoded limit M__ (" + std::to_string(M__) + ")");
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            if (j < dist_json->array.size() && i < dist_json->array[j].array.size()) {
                Ob_Base_dist[i][j] = dist_json->array[j].array[i].number; 
            } else {
                throw std::runtime_error("Distance matrix dimensions do not match sites/offices");
            }
        }
    }

    // Параметры задачи
    gap_treshold = (int)root.find("maxDifference")->number;
    float balanceCoeff = root.find("balanceCoefficient")->number;

    // 4 фиктивных офиса по углам
    float stepaway = 4.0;
    Base_x[m+0] = xmin - (xmax-xmin)*stepaway/2; Base_y[m+0] = ymin - (ymax-ymin)*stepaway/2;
    Base_x[m+1] = xmax + (xmax-xmin)*stepaway/2; Base_y[m+1] = ymin - (ymax-ymin)*stepaway/2;
    Base_x[m+2] = xmin - (xmax-xmin)*stepaway/2; Base_y[m+2] = ymax + (ymax-ymin)*stepaway/2;
    Base_x[m+3] = xmax + (xmax-xmin)*stepaway/2; Base_y[m+3] = ymax + (ymax-ymin)*stepaway/2;
    
    for (int i = 0; i < 4; ++i) {
        Base_weight[m + i] = 0;
        Base_nodes_max[m + i] = 1000000; // Чтобы не влиять на баланс
    }
    
    m += 4;
	const float BIG = 1e18f;
	for (int i = 0; i < n; ++i) {
		for (int k = 0; k < 4; ++k) {
			Ob_Base_dist[i][m - 4 + k] = BIG;
		}
	}

    global_rad_x = (xmax - xmin) / sqrt(n);
    global_rad_y = (ymax - ymin) / sqrt(n);
}

void save_solution_to_json(const char* filename, int real_m) {
    std::ofstream out(filename);
    out << "[\n";
    bool first_office = true;

    for (int i = 0; i < real_m; ++i) {
        if (Base_start[i] != -1) {
            if (!first_office) out << ",\n";
            first_office = false;

            out << "  {\n";
            out << "    \"officeId\": " << i << ",\n";
            
            // Собираем сайты
            out << "    \"sites\": [";
            int g = Base_start[i];
            bool first_site = true;
            while (g != -1) {
                if (!first_site) out << ", ";
                first_site = false;
                out << g;
                if (g == Base_finish[i]) break;
                g = Ob_forward[g];
            }
            out << "],\n";

            out << "    \"workers\": [";
            for (int j = 0; j < Base_max[i]; ++j) {
                if (j > 0) out << ", ";
                out << Base_Worker[i][j];
            }
            out << "]\n";
            out << "  }";
        }
    }
    out << "\n]\n";
}

void prepare_objective() {
	initial_summ=0;
	for (int i=0;i<n;i++) {
		initial_summ+=Ob_weight[i];
	}
}

int main(int argc, char** argv) {


	coeffs[0]=0.1;coeffs_min[0]=0.0001;coeffs_max[0]=0.5;
	coeffs[1]=0.95;coeffs_min[1]=0.5;coeffs_max[1]=0.99;
	coeffs[2]=1.0;coeffs_min[2]=0.01;coeffs_max[2]=10.0;
	coeffs[3]=1.0;coeffs_min[4]=0.01;coeffs_max[3]=10.0;
	coeffs[4]=0.02;coeffs_min[5]=0.001;coeffs_max[4]=0.5;
	
	coeffs[5]=1;coeffs_min[5]=0;coeffs_max[5]=2;
	coeffs[6]=1;coeffs_min[6]=0;coeffs_max[6]=2;
	coeffs[7]=1;coeffs_min[7]=0;coeffs_max[7]=2;
	coeffs[8]=1;coeffs_min[8]=0;coeffs_max[8]=2;
	coeffs[9]=1;coeffs_min[9]=0;coeffs_max[9]=2;
	
	for (int i=0;i<10;i++) {
		for (int j=0;j<qu_coeffs;j++) {
			coeffs_density[i][j]=0.2/(qu_coeffs-1);
		}
		coeffs_density[i][(int)(coeffs[i]*qu_coeffs/(coeffs_max[i]-coeffs_min[i]))]=0.8;
	}

	std::string input_path  = "instance.json";
    std::string output_path = "solution.json";

	// ДЛЯ ПЕРЕДАЧИ НАЗВАНИЯ ФАЙЛА С ТЕСТОМ НА ВХОД КАК ФЛАГ
#ifdef CLI_FILE_MODE
	if (argc > 1) {
		input_path = argv[1];
		size_t dot_pos   = input_path.find_last_of('.');
		size_t slash_pos = input_path.find_last_of("/\\");
		bool has_ext = (dot_pos != std::string::npos) &&
						(slash_pos == std::string::npos || dot_pos > slash_pos);
		if (has_ext) {
			output_path = input_path.substr(0, dot_pos)
						+ "_solution"
						+ input_path.substr(dot_pos);
		} else {
			output_path = input_path + "_solution.json";
		}
	}
	if (argc > 2) {
		output_path = argv[2];
	}
#endif
	
	int ttt=time(0);
#ifdef INTERACTIVE_MODE
	//ttt=1790337119;
	//to check bad testcases
#endif
	srand(ttt);
	
	int real_m = 0;
	
#ifdef INTERACTIVE_MODE
	// Старый интерактивный ввод
	DBG("seed:%d\n",ttt);
	int lx,ly;
	DBG("N,M,W,dx,dy:");
	scanf("%d %d %d %d %d",&n,&m,&wo,&lx,&ly);
	DBG("Nodes(gap). -1 ~ no constraint:");
	scanf("%d",&gap_treshold);
	float p1,p2;
	DBG("P[tasks avaiable],P[tasks has]:");
	scanf("%f %f",&p1,&p2);
	
	time_final=time(0)+290;
	LN=2*(int)(log(n)/log(2)+5);
	
	DBG("#");
	sample_generate(n,m,wo,lx,ly,4,p1,p2);
	DBG("#");
	
	real_m = m - 4;
#else
	std::string text = read_file(input_path.c_str());
	std::size_t pos = 0;
	Json root = Json::parse(text, pos);
	load_instance_from_json(root);
	
	time_final=time(0)+29;
	LN=2*(int)(log(n)/log(2)+5);
	
	real_m = m - 4;
#endif
	
	Qtree_generate();
	DBG("#");
	Steiner_generate();
	DBG("#");
	Steiner_prepare_constraints();
	DBG("#");
	determine_class_prepare();
	DBG("#");
	determine_class_forall();
	DBG("#");
	
	prepare_objective();
	
	for (int i=0;i<n;i++) {Wanna[i]=Current[i];}
	
	// Инициализация состояния офисов на случай, если алгоритм не найдёт решение
	for (int i = 0; i < m; i++) {
		Base_start[i] = -1;
		Base_finish[i] = -1;
		Base_max[i] = 0;
	}
	
	Steiner_save(0);
	Steiner_save(1);
	Steiner_save(2);
	routes_save(2);
	
	int uf_best=-1,uf1_best=-1,uf2_best=-1,uf3_best=-1,uf4_best=-1;
	float penalty=0;
	double global_best_found=-1;
	
	float len;
	int tick=0,uf,uf1,uf2,uf3;
	int uf_=0,uf1_=0,uf2_=0,uf3_=0;
	float po=1;
	int descend=10;
	
	while (is_time_still()) {
		variate_coefficients();
		
		penalty=tick*tick+po;
		tick+=1;
		po*=3;
		
		DBG("running G block:");
		geometry_variate(10,10,10,50);
		DBG("\n");
		DBG("running R block:");
		routes_variate(10,10,penalty, coeffs[0],2, coeffs[4]);
		DBG("\n");
		DBG("getting feedback\n");
		routes_color();
		DBG("got feedback\n");
		
		len=evaluate_routes(penalty);
		uf=evaluate_unfeasibility();
		uf1=evaluate_unfeasibility_Nodes();
		uf2=0;
		uf3=evaluate_unfeasibility_Skills();
		
		DBG("price:%f penalty_border:%f unfeasibility:%d %d %d %d   |before restart:%d\n",
			len,penalty,uf,uf1,uf2,uf3,descend);
		
		if ((uf>=0) && ((uf<uf_)||(uf1<uf1_)||(uf2<uf2_)||(uf3<uf3_))) {
			if ((uf<uf_)&&(uf1<uf1_)&&(uf2<uf2_)&&(uf3<uf3_)) {descend+=5;}
			coeffs_feedback(0.1);
		} else {
			coeffs_feedback(-0.1);
		}
		
		descend--;
		
		uf_=uf; uf1_=uf1; uf2_=uf2; uf3_=uf3;
		
		if (((uf<0)||((uf<=0)&&(uf1<=0)&&(uf2<=0)&&(uf3<=0))) || (descend<=0)) {
			if (descend<=0) {
				descend=15;
				Steiner_load(2);
				routes_load(2);
			}
			tick=2;
			po=1;
			DBG("-->Autorestart\n");
			coeffs_feedback(1);
		}
		
		if (global_best_found<0) {
			if (((uf_best<0)||(uf<uf_best)) && ((uf1_best<0)||(uf1<uf1_best)) &&
				((uf2_best<0)||(uf2<uf2_best)) && ((uf3_best<0)||(uf3<uf3_best))) {
				uf_best=uf; uf1_best=uf1; uf2_best=uf2; uf3_best=uf3;
				DBG("Have not found feasible still, that is why saving unfeasible\n");
				Steiner_save(2);
				routes_save(2);
			}
		}
		
		if ((len>=0)&&(uf==0)&&(uf1==0)&&(uf2==0)&&(uf3==0) &&
			((global_best_found<0) || (len<global_best_found))) {
			DBG("------>New solution with %f price has been found!!! | without upgradeing at all<-------\n",len);
			descend=10;
			coeffs_feedback(0.5);
			len=evaluate_routes(penalty);
			global_best_found=len;
			Steiner_save(2);
			routes_save(2);
		}
		
		if ((rand()%(int)(sqrt(tick)+1)==0)||(tick==1)) {
			DBG("trying to prepare it:");
			routes_variate(10,10,-1, coeffs[0],2, coeffs[4]);
			DBG("\n");
			float tem=evaluate_routes(-1);
			if ((tem>=0)&&((global_best_found<0)||(tem<global_best_found))) {
				if ((evaluate_unfeasibility()==0)&&(evaluate_unfeasibility_Nodes()==0)&&(evaluate_unfeasibility_Skills()==0)) {
					DBG("------>New solution with %f price has been found!!! | small upgrades were made<-------\n",tem);
					global_best_found=tem;
					Steiner_save(2);
					routes_save(2);
					descend=10;
					coeffs_feedback(10);
					DBG("trying to upgrade it:");
					routes_variate(10,10,-1, coeffs[0],2, coeffs[4]);
					DBG("\n");
				} else {
					DBG("[%d %d %d]\n",
						evaluate_unfeasibility(),
						evaluate_unfeasibility_Nodes(),
						evaluate_unfeasibility_Skills());
					DBG("The solution was unfeasible, that is why won't be observed\n");
				}
			}
		}
	}
	
	Steiner_load(2);
	routes_load(2);
	
	// Сохраняем результат всегда. В интерактивном режиме — для проверки формата,
	// в submission — это и есть ответ.
	save_solution_to_json(output_path.c_str(), real_m);
	
	QTree_destroy();
	return 0;
}
