#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

void err_exit(const char* const function_name) {
	perror(function_name);
	exit(EXIT_FAILURE);
}

#ifdef _WIN32
	#include <windows.h>
	#include <conio.h>
#else
	#include <errno.h>
	#include <signal.h>
	#include <termios.h>
	#include <unistd.h>

	// https://viewsourcecode.org/snaptoken/kilo/02.enteringRawMode.html

	void tcsetattr_exit_on_err(
		int fd,
		int optional_actions,
		const struct termios *termios_p
	) {
		if (tcsetattr(fd, optional_actions, termios_p) != 0) err_exit("tcsetattr");
		// https://linux.die.net/man/3/tcgetattr:
		// Note that tcsetattr() returns success if any of the requested changes
		// could be successfully carried out. Therefore, when making multiple
		// changes it may be necessary to follow this call with a further call to
		// tcgetattr() to check that all changes have been performed successfully.
		struct termios tmp;
		if (tcgetattr(fd, &tmp) != 0) err_exit("tcgetattr (check after tcsetattr)");
	}

	struct termios orig_termios;
	bool in_raw_mode = false;

	// maybe use /dev/tty instead of STDIN_FILENO

	// this is not really a raw mode, maybe rename
	void enable_raw_mode() {
		assert(!in_raw_mode);
		if (tcgetattr(STDIN_FILENO, &orig_termios) != 0) err_exit("tcgetattr");
		struct termios raw = orig_termios;
		raw.c_lflag &= ~(ECHO | ICANON);
		raw.c_cc[VMIN] = 0;
		raw.c_cc[VTIME] = 0;
		in_raw_mode = true;
		// maybe use TCSADRAIN instead
		tcsetattr_exit_on_err(STDIN_FILENO, TCSAFLUSH, &raw);
	}
	void disable_raw_mode_assume_raw() {
		// maybe use TCSADRAIN instead
		tcsetattr_exit_on_err(STDIN_FILENO, TCSAFLUSH, &orig_termios);
		in_raw_mode = false;
	}
	void disable_raw_mode() {
		assert(in_raw_mode);
		disable_raw_mode_assume_raw();
	}

	void interrupt_handler(const int signal_number) {
		exit(signal_number);
	}
#endif


const int N=10000;
const int N_=1000;
const int Tmax=7;
int LN=2*(int)(log(N)/log(2)+5);
const int M=50;
const int M_=M*4;
const int M__=500;
const float pi=3.14159265;

float similar_rad=0.00001;

float Length_min=-1;
float Length_max=-1;
int Nodes_min=10;
int Nodes_max=200;

float Ob_x[N],Ob_y[N];
bool Ob_days[N][Tmax];
float Base_x[M],Base_y[M];
int Base_day[M][M__];
int Ob_Base[N];
int Base_Ob[M];
int Ob_Neighbors[N][N_],Ob_Neighbors_max[N];
float Ob_Neighbors_weights[N][N_];
int Ob_back[N],Ob_forward[N];//to store a path
int Base_start[M][M__],Base_finish[M][M__],Base_max[M];//to store path begin and end
float Base_length[M][M__];
int Base_nodes[M][M__];
int Base_nodes_days[M][M__];
int Ob_back_save[N][3],Ob_forward_save[N][3];//to store a path - saved
int Base_start_save[M][M__][3],Base_finish_save[M][M__][3],Base_max_save[M][3],Base_day_save[M][M__][3];//to store path begin and end - saved
int m=0,n=0;

bool base_stays_on_cluster=true;

int Ob_freed[N],Ob_freed_max;
bool Base_need_repair[M][M__];

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

float aim_x,aim_y;

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

void movepos(short x, short y) {
#ifdef _WIN32
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD position = { x, y };
	SetConsoleCursorPosition(hStdout, position);
#else
	printf("\x1B[%d;%dH", y + 1, x + 1);
#endif
}

void clear_terminal_screen() {
#ifdef _WIN32
	system("cls");
#else
	printf("\x1B[3J\x1B[H");
#endif
}

void ListStorage() {
	printf("Tree:\n");
	for (int i=0;i<QTree_max;i++) {
		printf("%d:%d < %c < %d  - %d\n",i,QTree_small[i],QTree_variable[i],QTree_big[i],QTree_Storage[i]);
	}
	printf("Storages:\n");
	for (int i=0;i<Storage_max_max;i++) {
		printf("%d:",i);
		for (int j=0;j<Storage_max[i];j++) {
			printf(" %d ",Storage[i][j]);
		}
		printf("\n");
	}
	
}

void ListSteiner() {
	printf("vertex:\n");
	for (int i=0;i<ms;i++) {
		printf("%2d :",i);
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			printf("%2d [%d %d] ",SteinerNeighbors[i][j],SteinerNeighbors_R[i][j],SteinerNeighbors_L[i][j]);
		}printf(":\n");
	}
	printf("coords:\n");
	for (int i=0;i<ms;i++) {
		printf("%d : %.2f %.2f\n",i,Steiner_x[i],Steiner_y[i]);
	}
}

void Draw_new(int dx,int dy) {
	for (int y=0;y<screen_y;y+=dy) {
		for (int x=0;x<screen_x;x+=dx) {
			if (screen[x][y][0]!=screen_[x][y][0]) {
				movepos(x/dx,y/dy+1);
				printf("%c",screen[x][y][0]);
				screen_[x][y][0]=screen[x][y][0];
			}
		}
		for (int x=0;x<screen_x;x+=dx) {
			if (screen[x][y][1]!=screen_[x][y][1]) {
				movepos(x/dx+screen_x/dx+1,y/dy+1);
				printf("%c",screen[x][y][1]);
				screen_[x][y][1]=screen[x][y][1];
			}
		}
	}
}

void draw_routes() {
	for (int i=0;i<m-4;i++) {
		printf("Base %d:                 \n",i);
		for (int j=0;j<Base_max[i];j++) {
			int g;
			
			printf("    Route %d (day %d) ->:",j,Base_day[i][j]);
			g=Base_start[i][j];
			while ((g!=Base_finish[i][j])&&(g!=-1)) {
				//printf("-[%d %d]-",g,Current[g]);
				printf("[%d]",g);
				//for (int ii=0;ii<Tmax;ii++) printf("%c",'A'+Ob_days[g][ii]);
				g=Ob_forward[g];
			}
			if (g!=-1) {
				//printf("-[%d %d]-",g,Current[g]);
				printf("[%d]",g);
				//for (int ii=0;ii<Tmax;ii++) printf("%c",'A'+Ob_days[g][ii]);
				if (g==Base_finish[i][j]) {printf(";");}
			} else {
				printf("|");
			}
			printf("                          \n");
			
			/*printf("    Route %d <-:",j);
			g=Base_finish[i][j];
			while ((g!=Base_start[i][j])&&(g!=-1)) {
				printf("-[%d %d]-",g,Current[g]);
				g=Ob_back[g];
			}
			if (g!=-1) {
				printf("-[%d %d]-",g,Current[g]);
			} else {
				printf("|");
			}
			printf("                          \n");/**/
		}
	}
}

void routes_save(int mode) {
	for (int i=0;i<n;i++) {
		Ob_back_save[i][mode]=Ob_back[i];
		Ob_forward_save[i][mode]=Ob_forward[i];
	}
	for (int i=0;i<m;i++) {
		Base_max_save[i][mode]=Base_max[i];
		for (int j=0;j<Base_max[i];j++) {
			Base_start_save[i][j][mode]=Base_start[i][j];
			Base_finish_save[i][j][mode]=Base_finish[i][j];
			Base_day_save[i][j][mode]=Base_day[i][j];
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
		for (int j=0;j<Base_max_save[i][mode];j++) {
			Base_start[i][j]=Base_start_save[i][j][mode];
			Base_finish[i][j]=Base_finish_save[i][j][mode];
			Base_day[i][j]=Base_day_save[i][j][mode];
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
	float Base_x_[M],Base_y_[M];
	float dx[M],dy[M],mx[M],my[M],ddx,ddy,ddr,ddx_,ddy_,ddr_,angle[M];
	int imin,j_,a,b,vlink[M_],A,B;
	float vx[M_],vy[M_],vmax,t;
	bool valid[M],isit;
	
	for (int i=0;i<m;i++) {//doing for i
		Base_x_[i]=Base_x[i]+rand_huge()%100000/100000.0-0.5;
		Base_y_[i]=Base_y[i]+rand_huge()%100000/100000.0-0.5;
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
				//tt[dxdx+dydy]-2t[ dy[mx-xi]-dx[my-yi] ]+[mx-xi]2+[my-yi]2
				t=( dy[j]*(mx[j]-Base_x_[i])-dx[j]*(my[j]-Base_y_[i]) ) / ( dx[j]*dx[j]+dy[j]*dy[j] );
				
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
		
		for (int j=0;j<m;j++) {
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
		
		/*for (int j=0;j<m-1;j++) {
			printf("[%.2f %.2f %.2f %.2f]\n",dx[j],dy[j],mx[j],my[j]);
		}printf("{%.2f %.2f}\n\n",Base_x[i],Base_y[i]);/**/
		
		vmax=0;
		a=0;
		b=0;
		j_=0;
		for (int j=0;j<m-1;j++) {
			
			if (valid[j]==1) {
				/*x*dx[j]+y*dy[j]=dx[j]*mx[j]+dy[j]*my[j]
				x*dx[j_]+y*dy[j_]=dx[j_]*mx[j_]+dy[j_]*my[j_]
				
				x*dx[j]*dx[j_]+y*dy[j]*dx[j_]=dx[j]*dx[j_]*mx[j]+dy[j]*dx[j_]*my[j]
				x*dx[j_]*dx[j]+y*dy[j_]*dx[j]=dx[j_]*dx[j]*mx[j_]+dy[j_]*dx[j]*my[j_]
				
				x*dx[j]*dy[j_]+y*dy[j_]*dy[j]=dx[j]*dy[j_]*mx[j]+dy[j]*dy[j_]*my[j]
				x*dx[j_]*dy[j]+y*dy[j_]*dy[j]=dx[j_]*dy[j]*mx[j_]+dy[j_]*dy[j]*my[j_]*/
				
				while ((a>0)&&( (vx[a-1]-mx[j])*dx[j] + (vy[a-1]-my[j])*dy[j] < 0 )) {
					a--;
					valid[vlink[a]]=0;
					//printf("@");
				}
				
				j_=j-1;
				if (j_<0) {j_+=m-1;}
				while (valid[j_]==0) {
					j_--;
					if (j_<0) {j_+=m-1;}
				}
				
				vx[a]=( dy[j]*dy[j_]*(my[j]-my[j_]) + dx[j]*dy[j_]*mx[j] - dx[j_]*dy[j]*mx[j_] ) / (dx[j]*dy[j_]-dx[j_]*dy[j]);
				vy[a]=( dx[j]*dx[j_]*(mx[j]-mx[j_]) + dy[j]*dx[j_]*my[j] - dy[j_]*dx[j]*my[j_] ) / (dy[j]*dx[j_]-dy[j_]*dx[j]);
				vlink[a]=j;
				
				//printf("cross[ %.2f %.2f with %.2f %.2f ] obtain [%.2f %.2f]\n",mx[j],my[j] ,mx[j_],my[j_] ,vx[a],vy[a]);
				
				a++;
			}
		}
		
		
		vmax=0;//firstly identify all who can be left
		a=0;
		b=0;
		j_=0;
		for (int j=0;j<m-1;j++) {
			
			if (valid[j]==1) {
				/*x*dx[j]+y*dy[j]=dx[j]*mx[j]+dy[j]*my[j]
				x*dx[j_]+y*dy[j_]=dx[j_]*mx[j_]+dy[j_]*my[j_]
				
				x*dx[j]*dx[j_]+y*dy[j]*dx[j_]=dx[j]*dx[j_]*mx[j]+dy[j]*dx[j_]*my[j]
				x*dx[j_]*dx[j]+y*dy[j_]*dx[j]=dx[j_]*dx[j]*mx[j_]+dy[j_]*dx[j]*my[j_]
				
				x*dx[j]*dy[j_]+y*dy[j_]*dy[j]=dx[j]*dy[j_]*mx[j]+dy[j]*dy[j_]*my[j]
				x*dx[j_]*dy[j]+y*dy[j_]*dy[j]=dx[j_]*dy[j]*mx[j_]+dy[j_]*dy[j]*my[j_]*/
				
				while ((a>0)&&( (vx[a-1]-mx[j])*dx[j] + (vy[a-1]-my[j])*dy[j] < 0 )) {
					a--;
					valid[vlink[a]]=0;
					//printf("@");
				}
				
				j_=j-1;
				if (j_<0) {j_+=m-1;}
				while (valid[j_]==0) {
					j_--;
					if (j_<0) {j_+=m-1;}
				}
				
				vx[a]=( dy[j]*dy[j_]*(my[j]-my[j_]) + dx[j]*dy[j_]*mx[j] - dx[j_]*dy[j]*mx[j_] ) / (dx[j]*dy[j_]-dx[j_]*dy[j]);
				vy[a]=( dx[j]*dx[j_]*(mx[j]-mx[j_]) + dy[j]*dx[j_]*my[j] - dy[j_]*dx[j]*my[j_] ) / (dy[j]*dx[j_]-dy[j_]*dx[j]);
				vlink[a]=j;
				
				//printf("cross[ %.2f %.2f with %.2f %.2f ] obtain [%.2f %.2f]\n",mx[j],my[j] ,mx[j_],my[j_] ,vx[a],vy[a]);
				
				a++;
			}
		}
		
		int j=vlink[0];//do not forget to finish a cycle
		while ((a>0)&&( (vx[a-1]-mx[j])*dx[j] + (vy[a-1]-my[j])*dy[j] < 0 )) {
			a--;
			valid[vlink[a]]=0;
			//printf("@");
		}
		
		
		vmax=0;//now we are ready to build without all the intruders
		a=0;
		b=0;
		j_=0;
		for (int j=0;j<m-1;j++) {
			
			if (valid[j]==1) {
				/*x*dx[j]+y*dy[j]=dx[j]*mx[j]+dy[j]*my[j]
				x*dx[j_]+y*dy[j_]=dx[j_]*mx[j_]+dy[j_]*my[j_]
				
				x*dx[j]*dx[j_]+y*dy[j]*dx[j_]=dx[j]*dx[j_]*mx[j]+dy[j]*dx[j_]*my[j]
				x*dx[j_]*dx[j]+y*dy[j_]*dx[j]=dx[j_]*dx[j]*mx[j_]+dy[j_]*dx[j]*my[j_]
				
				x*dx[j]*dy[j_]+y*dy[j_]*dy[j]=dx[j]*dy[j_]*mx[j]+dy[j]*dy[j_]*my[j]
				x*dx[j_]*dy[j]+y*dy[j_]*dy[j]=dx[j_]*dy[j]*mx[j_]+dy[j_]*dy[j]*my[j_]*/
				
				while ((a>0)&&( (vx[a-1]-mx[j])*dx[j] + (vy[a-1]-my[j])*dy[j] < 0 )) {
					a--;
					valid[vlink[a]]=0;
					//printf("@");
				}
				
				j_=j-1;
				if (j_<0) {j_+=m-1;}
				while (valid[j_]==0) {
					j_--;
					if (j_<0) {j_+=m-1;}
				}
				
				vx[a]=( dy[j]*dy[j_]*(my[j]-my[j_]) + dx[j]*dy[j_]*mx[j] - dx[j_]*dy[j]*mx[j_] ) / (dx[j]*dy[j_]-dx[j_]*dy[j]);
				vy[a]=( dx[j]*dx[j_]*(mx[j]-mx[j_]) + dy[j]*dx[j_]*my[j] - dy[j_]*dx[j]*my[j_] ) / (dy[j]*dx[j_]-dy[j_]*dx[j]);
				vlink[a]=j;
				
				//printf("cross[ %.2f %.2f with %.2f %.2f ] obtain [%.2f %.2f]\n",mx[j],my[j] ,mx[j_],my[j_] ,vx[a],vy[a]);
				
				a++;
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
			
			//printf("edge:%.2f %.2f - %.2f %.2f | %d - %d\n",Steiner_x[A],Steiner_y[A],Steiner_x[B],Steiner_y[B],A,B);
			
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
		
		/*for (int j=0;j<a;j++) {
			printf("[%f %f]\n",vx[j],vy[j]);
		}printf("{%f %f}\n\n",Base_x[i],Base_y[i]);/**/
		
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
		
		//printf("\n\n");
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
	float xa,ya,xb,yb,rad,t,aimx,aimy;
	float angle[M_];
	int angle_max=0;
	
	for (int i=0;i<ms;i++) {
		//finding out the best angle
		xa=Steiner_x[i];
		ya=Steiner_y[i];
		for (int j=0;j<SteinerNeighborsMax[i];j++) {
			xb=Steiner_x[SteinerNeighbors[i][j]];
			yb=Steiner_y[SteinerNeighbors[i][j]];
			angle[angle_max]=acos((xa-xb)/((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb)));
			if (ya-yb<0) {angle[angle_max]=2*pi-angle[angle_max];}
			angle_max++;
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
	aim_y=sin(aimx)*10;
	aim_x=cos(aimx)*10;
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
			if ((dx*aimx+dy*aimy)*(dx*aimx+dy*aimy)<(dx*dx+dy*dy)*(aimx*aimx+aimy*aimy)*0.00001) {
				j--;
				aimx=(rand_huge()%100000-50000)/10000.0;
				aimy=(rand_huge()%100000-50000)/10000.0;
			}
			if ((t>=0)&&(t<=1-0)) {
				dx=xa+dx*t-x;
				dy=ya+dy*t-y;
				rad=dx*dx+dy*dy;
				if ((radmin==-1)||(rad<radmin)) {
					radmin=rad;
					dx=xb-xa;
					dy=yb-ya;
					//<[x-xa,y-ya];[-dy,dx]> >= 0
					if ( -(x-xa)*dy+(y-ya)*dx >= 0 ) {
						ans=SteinerNeighbors_L[i][j];
					}
					if ( -(x-xa)*dy+(y-ya)*dx <= 0 ) {
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
	
	for (int i=0;i<n;i++) {
		x=(int)(Ob_x[i]/(float)dx+screen_x/2-movx/(float)dx);
		y=(int)(Ob_y[i]/(float)dy+screen_y/2-movy/(float)dy);
		if ((x>=0)&&(x<screen_x)&&(y>=0)&&(y<screen_y)) {
			aa=-2;
			while (aa==-2) {aa=determine_class(Ob_x[i],Ob_y[i]);}
			
			aa=aa+'0';
			if (aa>'0'+30) {aa='0'+30;}
			if (aa<'0'-10) {aa='0'-10;}
			
			if (aa=='0'-1) {
				aa=' ';
			}
			
			screen[x][y][screen_number]=aa;
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
				if (aa>'0'+30) {aa='0'+30;}
				if (aa<'0'-10) {aa='0'-10;}
				
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
		for (int j=0;j<Base_max[i];j++) {
			
			if (Base_start[i][j]!=-1) {
				draw_a_line(Base_x[i]/dx+screen_x/2-movx/dx,Base_y[i]/dy+screen_y/2-movy/dy,Ob_x[Base_start[i][j]]/dx+screen_x/2-movx/dx,Ob_y[Base_start[i][j]]/dy+screen_y/2-movy/dy,screen_number);
			}
			
			g=Base_start[i][j];
			while ((g!=Base_finish[i][j])&&(g!=-1)) {
				g_=g;
				g=Ob_forward[g];
				if ((g!=-1)&&(g_!=-1)) {draw_a_line(Ob_x[g]/dx+screen_x/2-movx/dx,Ob_y[g]/dy+screen_y/2-movy/dy,Ob_x[g_]/dx+screen_x/2-movx/dx,Ob_y[g_]/dy+screen_y/2-movy/dy,screen_number);}
			}
			
			if (Base_finish[i][j]!=-1) {
				draw_a_line(Base_x[i]/dx+screen_x/2-movx/dx,Base_y[i]/dy+screen_y/2-movy/dy,Ob_x[Base_finish[i][j]]/dx+screen_x/2-movx/dx,Ob_y[Base_finish[i][j]]/dy+screen_y/2-movy/dy,screen_number);
			}
			
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

void routes_greedy(float penalty) {//generates the greedy routes
	float xx[M][M__],yy[M][M__],dx,dy,length_has[M][M__];
	double r,minr;
	int last[M][M__],lead_id,lead_m,lead_mm,lead_freed,nodes_has[M][M__];
	for (int i=0;i<n;i++) {
		Ob_freed[i]=i;
		
		Ob_forward[i]=-1;
		Ob_back[i]=01;
	}
	Ob_freed_max=n;
	
	for (int i=0;i<m-4;i++) {//prepare for generating the routes
		for (int j=0;j<Base_max[i];j++) {
			xx[i][j]=Base_x[i];
			yy[i][j]=Base_y[i];
			Base_start[i][j]=-1;
			Base_finish[i][j]=-1;
			Base_day[i][j]=rand()%(Tmax-1)+1;
			last[i][j]=-1;
			nodes_has[i][j]=0;
			length_has[i][j]=0;
		}
	}
	
	bool cando;
	
	minr=0;
	while ((Ob_freed_max>0)&&(minr>=0)&&(is_time_still()==true)) {//for every base add such routes as requires
		minr=-1;
		for (int i=0;i<m-4;i++) {//guessing the vertice to connect
			for (int j=0;j<Base_max[i];j++) {
				for (int k=0;k<Ob_freed_max;k++) {
					if ((Current[Ob_freed[k]]==i)||(penalty>=0)) {
						cando=true;
						dx=xx[i][j]-Ob_x[Ob_freed[k]];
						dy=yy[i][j]-Ob_y[Ob_freed[k]];
						r=sqrt(dx*dx+dy*dy);
						
						if ((Current[Ob_freed[k]]!=i)&&(penalty>=0)) {r+=penalty;}
						
						if ((Ob_days[Ob_freed[k]][0]==false)&&(Ob_days[Ob_freed[k]][Base_day[i][j]]==false)) {
							if (penalty>=0) {r+=penalty;}//day 0 means it can be served at any time
							if (penalty<0) {cando=false;}
						}

						if ((Nodes_max>0)&&(nodes_has[i][j]+1>Nodes_max)) {
							if (penalty>=0) {r+=penalty;}
							if (penalty<0) {cando=false;}
						}
						
						dx=Base_x[i]-Ob_x[Ob_freed[k]];//connect current choosen with the finish point, it is the worst case.
						dy=Base_y[i]-Ob_y[Ob_freed[k]];
						if ((Length_max>0)&&(length_has[i][j]+r+sqrt(dx*dx+dy*dy)>Length_max)) {
							if (penalty>=0) {r+=penalty;}
							if (penalty<0) {cando=false;}
						}
						
						if (((minr<0)||(r<minr))&&(cando==true)) {
							minr=r;
							lead_id=Ob_freed[k];
							lead_m=i;
							lead_mm=j;
							lead_freed=k;
						}
					}
				}
			}
		}
		if (minr>=0) {
			dx=xx[lead_m][lead_mm]-Ob_x[lead_id];
			dy=yy[lead_m][lead_mm]-Ob_y[lead_id];
			r=sqrt(dx*dx+dy*dy);
			
			length_has[lead_m][lead_mm]+=r;
			nodes_has[lead_m][lead_mm]+=1;
			
			if (Base_start[lead_m][lead_mm]==-1) {
				Base_start[lead_m][lead_mm]=lead_id;
			}
			
			if (last[lead_m][lead_mm]!=-1) {
				Ob_forward[last[lead_m][lead_mm]]=lead_id;
				Ob_back[lead_id]=last[lead_m][lead_mm];
			} else {
				Ob_back[lead_id]=-1;
			}
			last[lead_m][lead_mm]=lead_id;
			
			xx[lead_m][lead_mm]=Ob_x[lead_id];
			yy[lead_m][lead_mm]=Ob_y[lead_id];
			
			Ob_freed[lead_freed]=Ob_freed[Ob_freed_max-1];
			Ob_freed_max--;
		}
	}
	
	for (int i=0;i<m-4;i++) {//closing the loop of the route
		for (int j=0;j<Base_max[i];j++) {
			Base_finish[i][j]=last[i][j];
		}
	}
}

void routes_greedy_fast(float penalty, float rad_x, float rad_y) {//generates the greedy routes
	float xx[M][M__],yy[M][M__],dx,dy,length_has[M][M__];
	double r,minr;
	int last[M][M__],lead_id,lead_m,lead_mm,lead_link,nodes_has[M][M__];
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
		Ob_back[i]=01;
	}
	Ob_freed_max=n;
	
	for (int i=0;i<m-4;i++) {//prepare for generating the routes
		for (int j=0;j<Base_max[i];j++) {
			xx[i][j]=Base_x[i];
			yy[i][j]=Base_y[i];
			Base_start[i][j]=-1;
			Base_finish[i][j]=-1;
			Base_day[i][j]=rand()%(Tmax-1)+1;
			last[i][j]=-1;
			nodes_has[i][j]=0;
			length_has[i][j]=0;
		}
	}
	
	bool cando;
	bool get_all_now=false;//stage 1 - fast, stage 2 - collect all the garbage which is still left and get it
	int id;
	
	minr=0;
	while ((QTree_elements[0]>0)&&(minr>=0)&&(is_time_still()==true)) {//for every base add such routes as requires
		minr=-1;
		
		for (int i=0;i<m-4;i++) {//guessing the vertice to connect
			for (int j=0;j<Base_max[i];j++) {
				
				for (int i=0;i<QTree_max;i++) {QTree_cond[i]=0;}
				
				if (get_all_now==true) {
					
					for (id=0;id<n;id++) {
						
						if (isfreed[id]==true) {
							
							//try to connect id exactly
							if ((Current[id]==i)||(penalty>=0)) {
								cando=true;
								dx=xx[i][j]-Ob_x[id];
								dy=yy[i][j]-Ob_y[id];
								r=sqrt(dx*dx+dy*dy);
								
								if ((Current[id]!=i)&&(penalty>=0)) {r+=penalty;}

								if ((Ob_days[id][0]==false)&&(Ob_days[id][Base_day[i][j]]==false)) {
									if (penalty>=0) {r+=penalty;}//day 0 means it can be served at any time
									if (penalty<0) {cando=false;}
								}

								if ((Nodes_max>0)&&(nodes_has[i][j]+1>Nodes_max)) {
									if (penalty>=0) {r+=penalty;}
									if (penalty<0) {cando=false;}
								}
								
								dx=Base_x[i]-Ob_x[id];//connect current choosen with the finish point, it is the worst case.
								dy=Base_y[i]-Ob_y[id];
								if ((Length_max>0)&&(length_has[i][j]+r+sqrt(dx*dx+dy*dy)>Length_max)) {
									if (penalty>=0) {r+=penalty;}
									if (penalty<0) {cando=false;}
								}

								if (((minr<0)||(r<minr))&&(cando==true)) {
									minr=r;
									lead_m=i;
									lead_mm=j;
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
										dx=xx[i][j]-Ob_x[id];
										dy=yy[i][j]-Ob_y[id];
										r=sqrt(dx*dx+dy*dy);

										if ((Current[id]!=i)&&(penalty>=0)) {r+=penalty;}

										if ((Nodes_max>0)&&(nodes_has[i][j]+1>Nodes_max)) {
											if (penalty>=0) {r+=penalty;}
											if (penalty<0) {cando=false;}
										}

										dx=Base_x[i]-Ob_x[id];//connect current choosen with the finish point, it is the worst case.
										dy=Base_y[i]-Ob_y[id];
										if ((Length_max>0)&&(length_has[i][j]+r+sqrt(dx*dx+dy*dy)>Length_max)) {
											if (penalty>=0) {r+=penalty;}
											if (penalty<0) {cando=false;}
										}

										if (((minr<0)||(r<minr))&&(cando==true)) {
											minr=r;
											lead_m=i;
											lead_mm=j;
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
										if ((xx[i][j]<QTree_treshold[run]+rad_x)||(QTree_elements[QTree_big[run]]==0)) {//let's observe it, may be useful
											QTree_cond[run]=1;
											run=QTree_small[run];
										} else {//No,let's try another
											QTree_cond[run]=1;
										}
									} else {
										if (QTree_variable[run]=='y') {
											if ((yy[i][j]<QTree_treshold[run]+rad_y)||(QTree_elements[QTree_big[run]]==0)) {//let's observe it, may be useful
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
											if ((xx[i][j]>=QTree_treshold[run]-rad_x)||(QTree_elements[QTree_small[run]]==0)) {//let's observe it, may be useful
												QTree_cond[run]=2;
												run=QTree_big[run];
											} else {//No,let's try another
												QTree_cond[run]=2;
											}
										} else {
											if (QTree_variable[run]=='y') {
												if ((yy[i][j]>=QTree_treshold[run]-rad_y)||(QTree_elements[QTree_small[run]]==0)) {//let's observe it, may be useful
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
		}
		
		if (minr>=0) {
			dx=xx[lead_m][lead_mm]-Ob_x[lead_id];
			dy=yy[lead_m][lead_mm]-Ob_y[lead_id];
			r=sqrt(dx*dx+dy*dy);
			
			length_has[lead_m][lead_mm]+=r;
			nodes_has[lead_m][lead_mm]+=1;
			
			if (Base_start[lead_m][lead_mm]==-1) {
				Base_start[lead_m][lead_mm]=lead_id;
			}
			
			if (last[lead_m][lead_mm]!=-1) {
				Ob_forward[last[lead_m][lead_mm]]=lead_id;
				Ob_back[lead_id]=last[lead_m][lead_mm];
			} else {
				Ob_back[lead_id]=-1;
			}
			last[lead_m][lead_mm]=lead_id;
			
			xx[lead_m][lead_mm]=Ob_x[lead_id];
			yy[lead_m][lead_mm]=Ob_y[lead_id];
			
			Ob_freed_max--;
			
			
			isfreed[lead_id]=false;
			
			run=lead_link;
			while (run!=0) {
				QTree_elements[run]--;
				run=QTree_back[run];
			}
			QTree_elements[run]--;
		}
		
		if ((minr<0)&&(get_all_now==false)) {//stage 2 - where we add all the rest without thinking of any optimization, just get all who is still left
			minr=0;//it is very important, because sometimes this optimization may give some vertices unvisited on the fast stage 1
			get_all_now=true;
		}
	}
	
	for (int i=0;i<m-4;i++) {//closing the loop of the route
		for (int j=0;j<Base_max[i];j++) {
			Base_finish[i][j]=last[i][j];
		}
	}
}

void route_break(int i,int j,int power) {
	int g_,g,l=0,a,b;
	
	g=Base_start[i][j];
	while ((g!=Base_finish[i][j])&&(g!=-1)) {
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
	
	l=0;
	g=Base_start[i][j];
	while ((g!=Base_finish[i][j])&&(g!=-1)) {
		g_=Ob_forward[g];
		
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
	
	if ((l>=a-1)&&(l<=b)) {Ob_forward[g]=-1;}
	if ((l>=a)&&(l<=b+1)) {Ob_back[g]=-1;}
	
	if ((l>=a)&&(l<=b)) {
		//if it is cutten, write it as freed
		Ob_freed[Ob_freed_max]=g;
		Ob_freed_max++;
	}
	
	if (a==0) {Base_start[i][j]=-1;}
	if (b>=l) {Base_finish[i][j]=-1;}
}

void routes_repair(float penalty,float T) {//T is being variated from 0 up to infinity, it is a coldness
	float xx_forward[M][M__],yy_forward[M][M__],xx_back[M][M__],yy_back[M][M__],dx,dy;
	double r,minr;
	int last_forward[M][M__],last_back[M][M__],lead_id,lead_m,lead_mm,lead_freed,g,g_;
	int forward_nodes[M][M__],back_nodes[M][M__];
	float forward_length[M][M__],back_length[M][M__];
	char lead_type;
	
	for (int i=0;i<m-4;i++) {//preparing for it, determining which routes needs any fixing
		for (int j=0;j<Base_max[i];j++) {
			
			forward_nodes[i][j]=0;
			back_nodes[i][j]=0;
			forward_length[i][j]=0;
			back_length[i][j]=0;
			
			last_forward[i][j]=-1;
			last_back[i][j]=-1;
			
			xx_forward[i][j]=Base_x[i];
			yy_forward[i][j]=Base_y[i];
			
			xx_back[i][j]=Base_x[i];
			yy_back[i][j]=Base_y[i];
			
			if ((Base_start[i][j]!=-1)&&(Base_finish[i][j]!=-1)) {//from both endpoints saved. Careful, it may be feasible!
				
				g=Base_start[i][j];
				while ((g!=-1)&&(g!=Base_finish[i][j])&&(is_time_still()==true)) {
					last_forward[i][j]=g;
					g=Ob_forward[g];
				}
				if (g!=-1) {//yep, it was feasible. Does not need to repair it anymore
					last_forward[i][j]=-2;
					last_back[i][j]=-2;
				} else {//nope, need to go from both sides to find a cutten ends
					g=Base_finish[i][j];
					while ((g!=-1)&&(g!=Base_start[i][j])) {
						last_back[i][j]=g;
						g=Ob_back[g];
					}
					
					xx_forward[i][j]=Ob_x[last_forward[i][j]];
					yy_forward[i][j]=Ob_y[last_forward[i][j]];
					xx_back[i][j]=Ob_x[last_back[i][j]];
					yy_back[i][j]=Ob_y[last_back[i][j]];
				}
				
			} else {
				
				if ((Base_start[i][j]!=-1)&&(Base_finish[i][j]==-1)) {//from one endpoint has been cutten, get ready to repair it
				
					g=Base_start[i][j];//start exists, go from it
					while (g!=-1) {
						last_forward[i][j]=g;
						g=Ob_forward[g];
					}
					
					xx_forward[i][j]=Ob_x[last_forward[i][j]];
					yy_forward[i][j]=Ob_y[last_forward[i][j]];
					xx_back[i][j]=Base_x[i];
					yy_back[i][j]=Base_y[i];
				
				} else {
					
					if ((Base_start[i][j]==-1)&&(Base_finish[i][j]!=-1)) {//from one endpoint has been cutten, get ready to repair it
						
						g=Base_finish[i][j];//finish exists, go from it
						while (g!=-1) {
							last_back[i][j]=g;
							g=Ob_back[g];
						}
						
						xx_forward[i][j]=Base_x[i];
						yy_forward[i][j]=Base_y[i];
						xx_back[i][j]=Ob_x[last_back[i][j]];
						yy_back[i][j]=Ob_y[last_back[i][j]];
						
					} else {
						
						if ((Base_start[i][j]==-1)&&(Base_finish[i][j]==-1)) {//both endpoints are missing
							//do nothing, there is nothing to save, the whole route will be rebuilt
							xx_forward[i][j]=Base_x[i];
							yy_forward[i][j]=Base_y[i];
							xx_back[i][j]=Base_x[i];
							yy_back[i][j]=Base_y[i];
						}
						
					}
					
				}
				
			}
			
		}
	}
	
	bool canadd;
	
	minr=0;
	while ((Ob_freed_max>0)&&(minr>=0)) {//for every base check its feasibility and if it is not true, rebuild
		minr=-1;
		for (int i=0;i<m-4;i++) {//prepare for generating the routes
			for (int j=0;j<Base_max[i];j++) {
				if (last_forward[i][j]!=-2) {//seems we may add somethig to it, it is not feasible
					for (int k=0;k<Ob_freed_max;k++) {
						
						if ((Current[Ob_freed[k]]==i)||(penalty>=0)) {
							canadd=true;
							dx=xx_forward[i][j]-Ob_x[Ob_freed[k]];//forward chain is being increased
							dy=yy_forward[i][j]-Ob_y[Ob_freed[k]];
							r=dx*dx+dy*dy;
							if (Current[Ob_freed[k]]!=i) {r+=penalty;}
							
							if ((Ob_days[Ob_freed[k]][0]==false)&&(Ob_days[Ob_freed[k]][Base_day[i][j]]==false)) {
								if (penalty>=0) {r+=penalty;}//day 0 means it can be served at any time
								if (penalty<0) {canadd=false;}
							}

							dx=xx_back[i][j]-Ob_x[Ob_freed[k]];//trying connect currently added with last from back chain, the worst case will be like this
							dy=yy_back[i][j]-Ob_y[Ob_freed[k]];
							if ((r+forward_length[i][j]+back_length[i][j]+sqrt(dx*dx+dy*dy)>Length_max)) {
								if (penalty>=0) {r+=penalty;}
								if (penalty<0) {canadd=false;}
							}
							
							if ((forward_nodes[i][j]+back_nodes[i][j]+1>Nodes_max)) {
								if (penalty>=0) {r+=penalty;}
								if (penalty<0) {canadd=false;}
							}
							
							if (
								(
									(minr<0)||(r<minr)||
									((r>minr)&&(rand_huge()%10000/10000.0<exp((minr-r)*T)))
								)&&(
								canadd==true
								)
							) {
								minr=r;
								lead_id=Ob_freed[k];
								lead_m=i;
								lead_mm=j;
								lead_freed=k;
								lead_type='f';
							}
							
							
							
							canadd=true;
							dx=xx_back[i][j]-Ob_x[Ob_freed[k]];//backward chain is being increased
							dy=yy_back[i][j]-Ob_y[Ob_freed[k]];
							r=dx*dx+dy*dy;
							if (Current[Ob_freed[k]]!=i) {r+=penalty;}
							
							if ((Ob_days[Ob_freed[k]][0]==false)&&(Ob_days[Ob_freed[k]][Base_day[i][j]]==false)) {
								if (penalty>=0) {r+=penalty;}//day 0 means it can be served at any time
								if (penalty<0) {canadd=false;}
							}

							dx=xx_forward[i][j]-Ob_x[Ob_freed[k]];//trying connect currently added with last from forward chain, the worst case will be like this
							dy=yy_forward[i][j]-Ob_y[Ob_freed[k]];
							if ((r+forward_length[i][j]+back_length[i][j]+sqrt(dx*dx+dy*dy)>Length_max)) {
								if (penalty>=0) {r+=penalty;}
								if (penalty<0) {canadd=false;}
							}
							
							if ((forward_nodes[i][j]+back_nodes[i][j]+1>Nodes_max)) {
								if (penalty>=0) {r+=penalty;}
								if (penalty<0) {canadd=false;}
							}
							
							if (
								(
									(minr<0)||(r<minr)||
									((r>minr)&&(rand_huge()%10000/10000.0<exp((minr-r)*T)))
								)&&(
								canadd==true
								)
							) {
								minr=r;
								lead_id=Ob_freed[k];
								lead_m=i;
								lead_mm=j;
								lead_freed=k;
								lead_type='b';
							}
						}
						
					}
				}
			}
		}
		if (minr>=0) {
			if (lead_type=='f') {//adding to the forward chain
				dx=xx_forward[lead_m][lead_mm]-Ob_x[lead_id];
				dy=yy_forward[lead_m][lead_mm]-Ob_y[lead_id];
				
				r=sqrt(dx*dx+dy*dy);
				forward_length[lead_m][lead_mm]+=r;
				forward_nodes[lead_m][lead_mm]++;
				
				if (last_forward[lead_m][lead_mm]==-1) {
					Base_start[lead_m][lead_mm]=lead_id;
				}
				if (last_forward[lead_m][lead_mm]!=-1) {
					Ob_forward[last_forward[lead_m][lead_mm]]=lead_id;
					Ob_back[lead_id]=last_forward[lead_m][lead_mm];
				} else {
					Ob_back[lead_id]=-1;
				}
				last_forward[lead_m][lead_mm]=lead_id;
				
				xx_forward[lead_m][lead_mm]=Ob_x[lead_id];
				yy_forward[lead_m][lead_mm]=Ob_y[lead_id];
			} else {//adding to the back chain
				dx=xx_back[lead_m][lead_mm]-Ob_x[lead_id];
				dy=yy_back[lead_m][lead_mm]-Ob_y[lead_id];
				
				r=sqrt(dx*dx+dy*dy);
				back_length[lead_m][lead_mm]+=r;
				back_nodes[lead_m][lead_mm]++;
				
				if (last_back[lead_m][lead_mm]==-1) {
					Base_finish[lead_m][lead_mm]=lead_id;
				}
				if (last_back[lead_m][lead_mm]!=-1) {
					Ob_back[last_back[lead_m][lead_mm]]=lead_id;
					Ob_forward[lead_id]=last_back[lead_m][lead_mm];
				} else {
					Ob_forward[lead_id]=-1;
				}
				last_back[lead_m][lead_mm]=lead_id;
				
				xx_back[lead_m][lead_mm]=Ob_x[lead_id];
				yy_back[lead_m][lead_mm]=Ob_y[lead_id];
			}
			
			Ob_freed[lead_freed]=Ob_freed[Ob_freed_max-1];
			Ob_freed_max--;
		}
	}
	
	if (Ob_freed_max==0) {//if we were succeed
		for (int i=0;i<m-4;i++) {//prepare for generating the routes
			for (int j=0;j<Base_max[i];j++) {
				if (last_forward[i][j]!=-2) {//was not feasible, we need do close both chains to make it a route
					if ((Base_start[i][j]!=-1)&&(Base_finish[i][j]!=-1)) {//both had something
						Ob_forward[last_forward[i][j]]=last_back[i][j];
						Ob_back[last_back[i][j]]=last_forward[i][j];
					} else {
						if ((Base_start[i][j]!=-1)&&(Base_finish[i][j]==-1)) {//one of them had something
							Base_finish[i][j]=last_forward[i][j];
						} else {
							if ((Base_start[i][j]==-1)&&(Base_finish[i][j]!=-1)) {//one of them had something
								Base_start[i][j]=last_back[i][j];
							} else {
								//both of them are empty, do nothing	
							}	
						}	
					}	
				}
			}
		}
	}
}

void days_repair() {
	int g,T;

	float days[Tmax];

	for (int i=0;i<m-4;i++) {
		for (int j=0;j<Base_max[i];j++) {

			for (int k=0;k<Tmax;k++) {days[k]=0;}
			T=0;

			g=Base_start[i][j];
			if (g!=-1) {

				while ((g!=Base_finish[i][j])&&(g!=-1)) {
					if (Ob_days[g][0]==false) {//detect day
						for (int k=1;k<Tmax;k++) {
							if (Ob_days[g][k]==true) {
								days[k]++;
								T++;
							}
						}
					}

					g=Ob_forward[g];
				}

				if (g!=-1) {
					if (Ob_days[g][0]==false) {//detect day
						for (int k=1;k<Tmax;k++) {
							if (Ob_days[g][k]==true) {
								days[k]++;
								T++;
							}
						}
					}
				}

			}

			if (T>0) {
				float ra=rand_huge()%10000/10000.0,ra_;
				for (int k=0;k<Tmax;k++) {
					ra_=ra;
					ra-=(float)(days[k])/(float)(T);
					if ((ra<=0)&&(ra_>=0)) {
						Base_day[i][j]=k;
					}
				}
			}

		}
	}
}

void routes_local_search_2swap(int iter_max) {
	bool didsth;
	float r1,r2,dx,dy;
	int g,g_,tt,tt1;
	for (int i=0;i<m-4;i++) {
			for (int j=0;j<Base_max[i];j++) {
				if ((Base_start[i][j]!=-1)&&(Base_finish[i][j]!=-1)&&(is_time_still()==true)) {//localsearch 2-swap
					didsth=true;
					
					for (int iiter=0;(iiter<=iter_max)&&(didsth==true);iiter++) {
						didsth=false;
						
						g=Base_start[i][j];//g goes from start to end
						while ((g!=Base_finish[i][j])&&(g!=-1)&&(didsth==false)) {
							if (Ob_forward[g]!=Base_finish[i][j]) {
								
								g_=Base_finish[i][j];//g_ goes from end to g
								while ((g_!=Base_start[i][j])&&(g_!=g)&&(Ob_back[g_]!=g)&&(g_!=-1)&&(didsth==false)) {
									if (Ob_back[g_]!=Ob_forward[g]) {
										r1=0;
										dx=Ob_x[g]-Ob_x[Ob_forward[g]];
										dy=Ob_y[g]-Ob_y[Ob_forward[g]];
										r1+=dx*dx+dy*dy;
										dx=Ob_x[g_]-Ob_x[Ob_back[g_]];
										dy=Ob_y[g_]-Ob_y[Ob_back[g_]];
										r1+=dx*dx+dy*dy;
										
										r2=0;
										dx=Ob_x[g]-Ob_x[Ob_back[g_]];
										dy=Ob_y[g]-Ob_y[Ob_back[g_]];
										r2+=dx*dx+dy*dy;
										dx=Ob_x[g_]-Ob_x[Ob_forward[g]];
										dy=Ob_y[g_]-Ob_y[Ob_forward[g]];
										r2+=dx*dx+dy*dy;
										
										if (r2<r1) {//need to make a swap
											didsth=true;//need restart
											
											tt1=Ob_forward[g];
											while (tt1!=g_) {
												tt=Ob_forward[tt1];
												Ob_forward[tt1]=Ob_back[tt1];
												Ob_back[tt1]=tt;
												
												tt1=Ob_back[tt1];
											}
											
											tt=Ob_forward[g];
											Ob_forward[g]=Ob_back[g_];
											Ob_back[g_]=tt;
											
											Ob_back[Ob_forward[g]]=g;
											Ob_forward[Ob_back[g_]]=g_;	
										}
										
									}
									if (didsth==false) {g_=Ob_back[g_];}
								}
								
							}
							if (didsth==false) {g=Ob_forward[g];}
						}
						
						if (didsth==false) {//now tro to swap the first edge with the base
							g=Base_start[i][j];
							while ((g!=Base_finish[i][j])&&(didsth==false)) {
								g_=Ob_forward[g];
								
								r1=0;
								dx=Ob_x[g_]-Ob_x[g];
								dy=Ob_y[g_]-Ob_y[g];
								r1+=dx*dx+dy*dy;
								dx=Ob_x[Base_start[i][j]]-Base_x[i];
								dy=Ob_y[Base_start[i][j]]-Base_y[i];
								r1+=dx*dx+dy*dy;
								
								r2=0;
								dx=Ob_x[g_]-Base_x[i];
								dy=Ob_y[g_]-Base_y[i];
								r2+=dx*dx+dy*dy;
								dx=Ob_x[Base_start[i][j]]-Ob_x[g];
								dy=Ob_y[Base_start[i][j]]-Ob_y[g];
								r2+=dx*dx+dy*dy;
								
								if (r2<r1) {
									didsth=true;
									tt1=Base_start[i][j];
									while (tt1!=g_) {
										tt=Ob_forward[tt1];
										Ob_forward[tt1]=Ob_back[tt1];
										Ob_back[tt1]=tt;
										
										tt1=Ob_back[tt1];
									}
									
									Ob_forward[Base_start[i][j]]=g_;
									Ob_back[g_]=Base_start[i][j];
									
									Base_start[i][j]=g;
								}
								g=g_;
							}
						}
						
						if (didsth==false) {//now tro to swap the first edge with the base
							g=Base_finish[i][j];
							while ((g!=Base_start[i][j])&&(didsth==false)) {
								g_=Ob_back[g];
								
								r1=0;
								dx=Ob_x[g_]-Ob_x[g];
								dy=Ob_y[g_]-Ob_y[g];
								r1+=dx*dx+dy*dy;
								dx=Ob_x[Base_finish[i][j]]-Base_x[i];
								dy=Ob_y[Base_finish[i][j]]-Base_y[i];
								r1+=dx*dx+dy*dy;
								
								r2=0;
								dx=Ob_x[g_]-Base_x[i];
								dy=Ob_y[g_]-Base_y[i];
								r2+=dx*dx+dy*dy;
								dx=Ob_x[Base_finish[i][j]]-Ob_x[g];
								dy=Ob_y[Base_finish[i][j]]-Ob_y[g];
								r2+=dx*dx+dy*dy;
								
								if (r2<r1) {
									didsth=true;
									tt1=Base_finish[i][j];
									while (tt1!=g_) {
										tt=Ob_forward[tt1];
										Ob_forward[tt1]=Ob_back[tt1];
										Ob_back[tt1]=tt;
										
										tt1=Ob_forward[tt1];
									}
									
									Ob_back[Base_finish[i][j]]=g_;
									Ob_forward[g_]=Base_finish[i][j];
									
									Base_finish[i][j]=g;
								}
								g=g_;
							}
						}
						
					}
				}
			}
		}
}

double evaluate_routes(float penalty) {
	int g,g_;
	int co;
	float dx,dy;
	double ans,ans_,ans_pen;
	
	if (Ob_freed_max>0) {return -1;}
	
	for (int i=0;i<m-4;i++) {
		for (int j=0;j<Base_max[i];j++) {
			
			ans_=0;
			ans_pen=0;
			
			Base_nodes_days[i][j]=0;

			g=Base_start[i][j];
			if (g==-1) {
				Base_nodes[i][j]=0;
				Base_length[i][j]=0;
				Base_nodes_days[i][j]=0;
			} else {
				dx=Base_x[i]-Ob_x[Base_start[i][j]];
				dy=Base_y[i]-Ob_y[Base_start[i][j]];
				ans_+=sqrt(dx*dx+dy*dy);
				
				if ((Current[Base_start[i][j]]!=i)&&(penalty>=0)) {ans_pen+=penalty;}
				if ((Ob_days[Base_start[i][j]][0]==false)&&(Ob_days[Base_start[i][j]][Base_day[i][j]]==false)) {Base_nodes_days[i][j]++;}
				
				Base_nodes[i][j]=1;
				
				while ((g!=Base_finish[i][j])&&(g!=-1)) {
					g_=g;
					g=Ob_forward[g];
					if (g!=-1) {
						dx=Ob_x[g]-Ob_x[g_];
						dy=Ob_y[g]-Ob_y[g_];
						ans_+=sqrt(dx*dx+dy*dy);
						if ((Current[g]!=i)&&(penalty>=0)) {ans_pen+=penalty;}
						if ((Ob_days[g][0]==false)&&(Ob_days[g][Base_day[i][j]]==false)) {Base_nodes_days[i][j]++;}
					}
					Base_nodes[i][j]++;
				}
				
				if ((g!=Base_finish[i][j])||(g==-1)) {return -1;}
				
				dx=Base_x[i]-Ob_x[Base_finish[i][j]];
				dy=Base_y[i]-Ob_y[Base_finish[i][j]];
				ans_+=sqrt(dx*dx+dy*dy);
				if ((Current[Base_finish[i][j]]!=i)&&(penalty>=0)) {ans_pen+=penalty;}
				
				Base_length[i][j]=ans_;
				
				co=Base_nodes[i][j];
				
				if ((co<Nodes_min)&&(Nodes_min>=0)&&(penalty>=0)) {ans_pen+=penalty*(Nodes_min-co);}
				if ((co>Nodes_max)&&(Nodes_max>=0)&&(penalty>=0)) {ans_pen+=penalty*(co-Nodes_max);}
				
				if ((ans_<Length_min)&&(Length_min>=0)&&(penalty>=0)) {ans_pen+=penalty*(Length_min-ans_);}
				if ((ans_>Length_max)&&(Length_max>=0)&&(penalty>=0)) {ans_pen+=penalty*(ans_-Length_max);}
				
				ans+=ans_+ans_pen;
			}
			
		}
	}
	return ans;
}

int evaluate_unfeasibility() {
	int g,ans=0;
	bool feas[N];
	for (int i=0;i<n;i++) {feas[i]=0;}
	for (int i=0;i<m-4;i++) {
		for (int j=0;j<Base_max[i];j++) {
			
			g=Base_start[i][j];
			if (g!=-1) {
				feas[g]=1;
				while ((g!=Base_finish[i][j])&&(g!=-1)) {
					if (Current[g]!=i) {ans++;}
					g=Ob_forward[g];
					feas[g]=1;
				}
				
				if ((g!=Base_finish[i][j])||(g==-1)) {
					return -1;
				}
				
				if (Current[Base_finish[i][j]]!=i) {ans++;}
			}
			
		}
	}
	for (int i=0;i<n;i++) {
		if (feas[i]==0) {return -1;}
	}
	return ans;
}

int evaluate_unfeasibility_Nodes() {//requires running evaluate_routes() before
	int ans=0,co;
	for (int i=0;i<m-4;i++) {
		for (int j=0;j<Base_max[i];j++) {
			co=Base_nodes[i][j];
			if ( ((co<Nodes_min)&&(Nodes_min>=0)) || ((co>Nodes_max)&&(Nodes_max>=0)) ) {ans++;}
		}
	}
	return ans;
}

int evaluate_unfeasibility_Lengths() {//requires running evaluate_routes() before
	int ans=0,co;
	for (int i=0;i<m-4;i++) {
		for (int j=0;j<Base_max[i];j++) {
			co=Base_length[i][j];
			if ( ((co<Length_min)&&(Length_min>=0)) || ((co>Length_max)&&(Length_max>=0)) ) {ans++;}
		}
	}
	return ans;
}

int evaluate_unfeasibility_Days() {//requires running evaluate_routes() before
	int ans=0;
	for (int i=0;i<m-4;i++) {
		for (int j=0;j<Base_max[i];j++) {
			ans+=Base_nodes_days[i][j];
		}
	}
	return ans;
}


void routes_variate(int iter_huge,int iter,float penalty,float cooling,float maxcool,float alpha) {//block R
	double goal,goal_best;
	float Temp;
	
	char doing;
	bool didsth;
	
	float distribution[M][M__];
	char distribution_type[M][M__];//to understand who is guilty and how
	
	for (int i=0;i<m;i++) {
		Base_max[i]=1;
	}
	
	float a,b;
	int a_num,b_num;
	
	float penalty_;

	int num;
	
	//routes_greedy(penalty);
	routes_greedy_fast(penalty,global_rad_x,global_rad_y);
	routes_save(0);
	goal_best=evaluate_routes(penalty);
	
	char per_prev=0;
	
	for (int iitt=0;(iitt<iter_huge)&&(is_time_still()==true);iitt++) {

		if (iitt<iter_huge/2) {
			penalty_=penalty*iitt/(iter_huge/2);
		} else {
			penalty_=penalty;
		}

		if (iitt*10/iter_huge>per_prev) {
			per_prev++;
			printf("[=]");
		}
		
		num=0;
		Temp=0;
		
		//routes_greedy(penalty_);
		routes_greedy_fast(penalty_,global_rad_x,global_rad_y);
		
		for (int i=0;i<m-4;i++) {
			num+=Base_max[i];
		}
		
		for (int i=0;i<m-4;i++) {
			for (int j=0;j<Base_max[i];j++) {
				distribution[i][j]=1.0/num;
			}
		}
		
		if (penalty<-1.5) {//looking for only a feasible solution, all the clusters behave independently but it is way too slow, penalty=-2 for this
			for (int i=0;i<m-4;i++) {
				for (int j=0;j<Base_max[i];j++) {
					distribution[i][j]=1.0/Base_max[i];
				}
			}
		}
		
		for (int tt=0;(tt<iter)&&(is_time_still()==true);tt++) {
			Temp+=cooling;
			if (Temp>maxcool) {Temp=0;}
			
			for (int i=0;i<m-4;i++) {
				for (int j=0;j<Base_max[i];j++) {
					if ((rand_huge()%10000)/10000.0<distribution[i][j]) {
						num=0;
						while ((rand()%10000/10000.0<coeffs[1])&&(num<n)) {
							num++;
						}
						route_break(i,j,2+num);
					}
				}
			}
			days_repair();
			routes_repair(penalty_,sqrt(Temp/cooling)*cooling);
			goal=evaluate_routes(penalty_);
			if ((goal>=0)&&(goal<goal_best)) {
				goal_best=goal;
				routes_save(0);
			}
			
			a=0;
			a_num=0;
			b=0;
			b_num=0;
			for (int i=0;i<m-4;i++) {
				for (int j=0;j<Base_max[i];j++) {
					distribution_type[i][j]=0;//did nothing bad
					if ((Base_length[i][j]>Length_max)&&(Length_max>=0)) {
						distribution_type[i][j]=1;
					}
					if (Base_length[i][j]<Length_min) {
						distribution_type[i][j]=2;
					}
					if ((Base_nodes[i][j]>Nodes_max)&&(Nodes_max>=0)) {
						distribution_type[i][j]=3;
					}
					if (Base_nodes[i][j]<Nodes_min) {
						distribution_type[i][j]=4;
					}
					if (Base_nodes_days[i][j]>0) {
						distribution_type[i][j]=5;
					}
					if (distribution_type[i][j]==0) {
						a+=distribution[i][j];
						a_num++;
					} else {
						b+=distribution[i][j];
						b_num++;
					}
				}
			}
			if ((a_num>0)&&(b_num>0)) {
				a/=a_num;
				b/=b_num;
				for (int i=0;i<m-4;i++) {
					for (int j=0;j<Base_max[i];j++) {
						if (distribution_type[i][j]==0) {
							distribution[i][j]=distribution[i][j]*(1-alpha)-alpha*b;
						} else {
							distribution[i][j]=distribution[i][j]*(1-alpha)+alpha*a;
						}
					}
				}
			}
			routes_load(0);
			Ob_freed_max=0;
			evaluate_routes(penalty_);
		}
		
		for (int i=0;i<m-4;i++) {
			doing=0;
			didsth=false;
			for (int j=0;j<Base_max[i];j++) {
				if (rand_huge()%100000/100000.0<distribution[i][j]) {
					if (
						((Base_nodes[i][j]>Nodes_max)&&(Nodes_max>=0))||
						((Base_length[i][j]>Length_max)&&(Length_max>=0))
					) {
						doing++;
						didsth=true;
					}
					if (
						(Base_nodes[i][j]<Nodes_min)||
						(Base_length[i][j]<Length_min)
					) {
						if (Base_max[i]+doing>1) {
							doing--;
						}
						didsth=true;
					}
				}
			}
			if (didsth==true) {
				Base_max[i]+=doing;
				if (Base_max[i]>=M__) {Base_max[i]=M__;}//just in case if something bad happens
				if (Base_max[i]==0) {Base_max[i]=1;}//just in case of something bad happens, so it should not
			}
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
			printf("[=]");
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
		for (int j=0;j<Base_max[i];j++) {
			g=Base_start[i][j];
			while ((g!=Base_finish[i][j])&&(g!=-1)) {
				Wanna[g]=i;
				g=Ob_forward[g];
			}
			Wanna[Base_finish[i][j]]=i;
		}
	}
}

void sample_generate(int nn,int mm,int wx,int wy,int stepaway,float prob_alldays,float prob_days) {
	n=nn;
	m=mm;
	float xmin=0,xmax=0,ymin=0,ymax=0;
	for (int i=0;i<m;i++) {
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
	
	for (int i=0;i<n;i++) {
		Ob_x[i]=wx*(rand_huge()%(100000))/100000.0-wx/2.0;
		Ob_y[i]=wy*(rand_huge()%(100000))/100000.0-wy/2.0;

		for (int j=0;j<Tmax;j++) {
			Ob_days[i][j]=false;
		}
		for (int j=0;j<Tmax;j++) {
			if (rand_huge()%100000/100000.0<prob_days) {Ob_days[i][j]=true;}
		}
		if (rand_huge()%100000/100000.0<prob_alldays) {
			Ob_days[i][0]=true;//can be visited at any day
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

int main() {
	
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
			coeffs_density[i][j]=0.2/(qu_coeffs-1);//start with the uniform
		}
		coeffs_density[i][(int)(coeffs[i]*qu_coeffs/(coeffs_max[i]-coeffs_min[i]))]=0.8;
	}
	
	char ch;
	
	int movx=0,movy=0;
	bool dodraw=false;
	
	int ttt=time(0);
	//ttt=1790337119;
	srand(ttt);
	
	printf("seed:%d\n",ttt);
	
	int lx,ly;
	
	printf("N,M,dx,dy:");
	scanf("%d %d %d %d",&n,&m,&lx,&ly);
	
	printf("Nodes(min,max),length(min,max). -1 ~ no constraint:");
	scanf("%d %d %f %f",&Nodes_min,&Nodes_max,&Length_min,&Length_max);
	
	float p1,p2;

	printf("P[always avaiable],P[avaiable at currect day]:");
	scanf("%f %f",&p1,&p2);

	time_final=time(0)+60*100;//setting a timer, set it to around 5 seconds before the timeout

	LN=2*(int)(log(n)/log(2)+5);
	
	sample_generate(n,m,lx,ly,5,p1,p2);//generating a sample
	
	Qtree_generate();//building a Qtree for further optimizatin
	
	Steiner_generate();//building a Steiner graph
	Steiner_prepare_constraints();//preparing a Steiner graph for variating
	
	determine_class_prepare();
	determine_class_forall();
	for (int i=0;i<n;i++) {Wanna[i]=Current[i];}//Setting everyone as they are
	
	Steiner_save(0);//saving all the Steiner stuff we have made as a good solution
	Steiner_save(1);
	
	clear_terminal_screen();
	
	int counting_bad,counting_bad_min=-1,iter=0;
	int uf_best=-1,uf1_best=-1,uf2_best=-1,uf3_best=-1,uf4_best=-1;
	float SAT_min=-1;
	ch=0;
	
	int size=1;
	float penalty=0;
	
	double global_best_found=-1;
	
	clear_terminal_screen();
	ScreenClear();
	
	dodraw=true;
	
	Steiner_save(2);
	routes_save(2);

#ifndef _WIN32
	struct sigaction sigint_action;
	sigint_action.sa_handler = &interrupt_handler;
	sigint_action.sa_flags = 0;
	if (sigfillset(&sigint_action.sa_mask) != 0)	  err_exit("sigfillset");
	if (sigaction(SIGINT, &sigint_action, NULL) != 0) err_exit("sigaction");

	enable_raw_mode();
	atexit(disable_raw_mode);
#endif

	while ((ch!=' ')&&(is_time_still()==true)) {

#ifndef _WIN32
		while (true) {
			char ch_buf[8];
			ssize_t n = read(STDIN_FILENO, ch_buf, sizeof ch_buf);
			if (n == 0) break;
			if (n == -1) {
				if (errno == EINTR) continue;
				else				err_exit("read");
			}
			for (ssize_t i = 0; i < n; ++i) {
				dodraw=true;
				ch = ch_buf[i];
				if (ch=='w') {movy-=5*size;}
				if (ch=='d') {movx+=5*size;}
				if (ch=='s') {movy+=5*size;}
				if (ch=='a') {movx-=5*size;}
				if (ch=='2') {size+=1;}
				if ((ch=='1')&&(size>=2)) {size-=1;}
				if (ch=='3') {
					movepos(0,60);
					draw_routes();
					dodraw=false;
				}
			}
			// The loop restarts for consistency with the windows version.
			// I would probably avoid doing so many read calls (at least 2 per
			// loop iteration) and just process 8 symbols per loop iteration.
		}
#else
		while (kbhit()) {
			dodraw=true;
			ch=getch();
			if (ch=='w') {movy-=5*size;}
			if (ch=='d') {movx+=5*size;}
			if (ch=='s') {movy+=5*size;}
			if (ch=='a') {movx-=5*size;}
			if (ch=='2') {size+=1;}
			if ((ch=='1')&&(size>=2)) {size-=1;}
			if (ch=='3') {
				movepos(0,60);
				draw_routes();
				dodraw=false;
			}
		}
#endif

		if (ch=='r') {
			Steiner_load(2);
			routes_load(2);
			
			clear_terminal_screen();
			printf("Stop - any,Continue - r\n");
			printf("starting calculations:\n");
			penalty=0;
			float len,len_min=-1;
			int tick=0,uf,uf1,uf2,uf3;
			int uf_=0,uf1_=0,uf2_=0,uf3_=0;
			ch='r';
			float po=1;
			while (((ch=='r')||(ch==0))&&(is_time_still()==true)) {
#ifndef _WIN32
				if (read(STDIN_FILENO, &ch, 1) == -1) {
					if (errno == EINTR) continue;
					else				err_exit("read");
				}
#else
				if (kbhit()) {
					ch=getch();
				}
#endif

				variate_coefficients();
				
				penalty=tick*tick+po;
				tick+=1;
				po*=1.5;
				
				printf("running G block:");
				geometry_variate(10,10,10,50);//O(mn)
				printf("\n");
				printf("running R block:");
				routes_variate(10,10,penalty, coeffs[0],2, coeffs[4]);//O(nn/m)
				printf("\n");
				printf("local searching R block:");
				routes_local_search_2swap(100);
				printf("\n");
				printf("getting feedback\n");
				routes_color();
				printf("got feedback\n");
				len=evaluate_routes(penalty);
				uf=evaluate_unfeasibility();
				uf1=evaluate_unfeasibility_Nodes();
				uf2=evaluate_unfeasibility_Lengths();
				uf3=evaluate_unfeasibility_Days();
				printf("price:%f penalty_border:%f unfeasibility:%d %d %d %d\n",len,penalty,uf,uf1,uf2,uf3);
				
				if (
					(uf>=0)&&
					(
						(uf<uf_)||
						(uf1<uf1_)||
						(uf2<uf2_)||
						(uf3<uf3_)
					)
				) {
					coeffs_feedback(0.1);
				} else {
					coeffs_feedback(-0.1);
				}
				
				uf_=uf;
				uf1_=uf1;
				uf2_=uf2;
				uf3_=uf3;

				if ((len_min<0)&&(len>0)) {
					len_min=len;//lower bound
				}
				
				if (
					((uf<0)||((uf<=0)&&(uf1<=0)&&(uf2<=0)&&(uf3<=0)))||
					((len>len_min*4)&&(len_min>=0))
				) {
					if ((len>len_min*4)&&(len_min>=0)) {
						Steiner_load(2);
						routes_load(2);
					}
					tick=2;
					po=1;
					printf("-->Autorestart\n");
					coeffs_feedback(1);
				}
				
				if (global_best_found<0) {//have not found yet
					if (
						((uf_best<0)||(uf<uf_best))&&
						((uf1_best<0)||(uf1<uf1_best))&&
						((uf2_best<0)||(uf2<uf2_best))&&
						((uf3_best<0)||(uf3<uf3_best))
					) {
						uf_best=uf;
						uf1_best=uf1;
						uf2_best=uf2;
						uf3_best=uf3;
						printf("Have not found feasible still, that is why saving unfeasible\n");
						Steiner_save(2);
						routes_save(2);
					}
				}

				if ((len>=0)&&(uf==0)&&(uf1==0)&&(uf2==0)&&(uf3==0)&&( (global_best_found<0) || (len<global_best_found) )) {
					printf("------>New solution with %f price has been found!!! | without upgradeing at all<-------\n",len);
					printf("local searching:");
					
					coeffs_feedback(10);
					
					routes_local_search_2swap(200);
					printf("\n");
					len=evaluate_routes(penalty);
					global_best_found=len;
					Steiner_save(2);
					routes_save(2);
				}
				
				if ((rand()%(int)(sqrt(tick)+1)==0)||(tick==1)) {
					printf("trying to prepare it:");
					routes_variate(10,10,-1, coeffs[0],2, coeffs[4]);
					printf("\n");
					routes_local_search_2swap(200);
					float tem=evaluate_routes(-1);
					if ((tem>=0)&&((global_best_found<0)||(tem<global_best_found))) {
						if ((evaluate_unfeasibility()==0)&&(evaluate_unfeasibility_Nodes()==0)&&(evaluate_unfeasibility_Lengths()==0)&&(evaluate_unfeasibility_Days()==0)) {
							printf("------>New solution with %f price has been found!!! | small upgrades were made<-------\n",tem);
							global_best_found=tem;
							Steiner_save(2);
							routes_save(2);
							
							coeffs_feedback(10);
							
							printf("trying to upgrade it:");
							routes_variate(10,10,-1, coeffs[0],2, coeffs[4]);
							printf("\n");
							routes_local_search_2swap(200);
						} else {
							printf("[%d %d %d %d]\n",evaluate_unfeasibility(),evaluate_unfeasibility_Nodes(),evaluate_unfeasibility_Lengths(),evaluate_unfeasibility_Days());
							printf("The solution was unfeasible, that is why won't be observed\n");
						}
					}
				}
			}
			Steiner_load(2);
			routes_load(2);
			
			determine_class_prepare();
			determine_class_forall();
			
			ch=0;
			
			clear_terminal_screen();
			ScreenClear();
		}
		
		if (dodraw==true) {
			ScreenPrepare(size,size,movx,movy,true,0);
			ScreenPrepare(size,size,movx,movy,false,1);
			ScreenPrepareRoutes(size,size,movx,movy,1);
			movepos(0,0);
			printf("wdsa - move, 12 - zoom, r - start calculations\n");
			Draw_new(1,1);
			dodraw=false;
		}
	}

	clear_terminal_screen();
	printf("Done in %d lag\n",time(0)-time_final);

	QTree_destroy();

	return 0;
}
