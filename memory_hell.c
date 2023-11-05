#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>

#define DT 0.05

typedef struct
{
    double x, y;
} vector;

int bodies, timeSteps, threadCount, localBodies;
double *masses, GravConstant;
vector *positions, *velocities, *accelerations, *forces;
sem_t *semsCompute, *semsPrint;
pthread_t *threadHandles;
FILE *out_txt, *out_csv; 

vector addVectors(vector a, vector b)
{
    vector c = {a.x + b.x, a.y + b.y};

    return c;
}

vector scaleVector(double b, vector a)
{
    vector c = {b * a.x, b * a.y};

    return c;
}

vector subtractVectors(vector a, vector b)
{
    vector c = {a.x - b.x, a.y - b.y};

    return c;
}

double mod(vector a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

void initiateSystem(char *fileName)
{
    int i;
    FILE *fp = fopen(fileName, "r");
    
    fscanf(fp, "%lf%d%d", &GravConstant, &bodies, &timeSteps);
    localBodies = bodies / threadCount; // количество тел в каждом потоке
    if(localBodies < 1) localBodies = 1; // на случай, если потоков больше, чем тел 
    
    threadHandles = (pthread_t *)malloc(threadCount * sizeof(pthread_t));
    semsCompute = (sem_t *)malloc((bodies * bodies) * sizeof(sem_t)); // синхронизация вычислений
    forces = (vector *)malloc((bodies * bodies) * sizeof(vector)); // хранение вычисленных сил
    semsPrint = (sem_t *)malloc((threadCount + 1) * sizeof(sem_t)); // синхронизация печати, [threadCount] будет использоваться для главного потока

    masses = (double *)malloc(bodies * sizeof(double));
    positions = (vector *)malloc(bodies * sizeof(vector));
    velocities = (vector *)malloc(bodies * sizeof(vector));
    accelerations = (vector *)malloc(bodies * sizeof(vector));

    for (i = 0; i < bodies; i++)
    {
        fscanf(fp, "%lf", &masses[i]);
        fscanf(fp, "%lf%lf", &positions[i].x, &positions[i].y);
        fscanf(fp, "%lf%lf", &velocities[i].x, &velocities[i].y);
    }

    fclose(fp);
}

void closeSystem()
{
    int i; 
    
    free(threadHandles);
    free(semsCompute);
    free(forces);
    free(semsPrint);
    free(masses);
    free(positions);
    free(velocities);
    free(accelerations);
}

int getRank(int body)
{
    // получить ранг потока по точке (не хотелось менять сигнатуры функций вычисления)
    int rank = body / localBodies;
    if (rank == threadCount) rank = threadCount - 1;
    return rank;
}

void computeAccelerations(int first, int last)
{
    int i, j;
    float denom;
    vector force;

    for (i = first; i < last; i++)
    {
        accelerations[i].x = 0;
        accelerations[i].y = 0;
        for (j = 0; j < bodies; j++)
        {
            // направление поменяно по сравнению с видео (от точек слева получаем силу, точки справа считаем)
            if (i == j) continue; // для самого себя не считаем
            if (i > j) // за нас посчитают
            {
               if (getRank(i) > getRank(j)) sem_wait(&semsCompute[j * bodies + i]);
               force = scaleVector(-1, forces[j * bodies + i]);
            }
            if (i < j) //считаем и сохраняем
            {
                denom = mod(subtractVectors(positions[i], positions[j]));
                if (denom < FLT_EPSILON) denom = FLT_EPSILON; // вместо ResolveCollisions
                force = scaleVector(GravConstant * masses[i] * masses[j] / pow(denom, 3), subtractVectors(positions[j], positions[i]));
                forces[i * bodies + j] = force;
                if (getRank(i) < getRank(j)) sem_post(&semsCompute[i * bodies + j]);
            }
            
            accelerations[i] = addVectors(accelerations[i], scaleVector(1 / masses[i], force));
        }
    }
}

void computeVelocities(int first, int last)
{
    int i;

    for (i = first; i < last; i++)
        velocities[i] = addVectors(velocities[i], scaleVector(DT, accelerations[i]));
}

void computePositions(int first, int last)
{
    int i;

    for (i = first; i < last; i++)
        positions[i] = addVectors(positions[i], scaleVector(DT,velocities[i]));
}

void printBody(int first, int last)
{
    int i;

    for (i = first; i < last; i++)
    {
        fprintf(out_txt, "Body %d : %lf\t%lf\t%lf\t%lf\n", i + 1, positions[i].x, positions[i].y, velocities[i].x, velocities[i].y);
        fprintf(out_csv, ",%lf,%lf", positions[i].x, positions[i].y);
    }
    
}

void* routine(void* rank)
{
    long myRank = (long)rank;
    // выбираем своё множество тел
    int myFirstBody = myRank * localBodies;
    int myLastBody = myFirstBody + localBodies;
    if (myLastBody > bodies) myLastBody = myFirstBody; // на случай, если потоков больше, чем тел
    if (myRank == threadCount - 1) myLastBody = bodies; // чтобы последний поток захватил остатки
    
    
    for (int i = 0; i < timeSteps; i++)
    {
        // считаем формулки
        computeAccelerations(myFirstBody, myLastBody);
        computePositions(myFirstBody, myLastBody);
        computeVelocities(myFirstBody, myLastBody);
        
        //проверяем, что нам можно печатать, печатаем и разрешаем следующему
        sem_wait(&semsPrint[myRank]);
        printBody(myFirstBody, myLastBody);
        sem_post(&semsPrint[myRank + 1]);
    }
    
    return NULL;
}

int main(int argc, char *argv[])
{
    int i, j;

    if (argc != 3) // используем два агрумента: файл ввода и число потоков
        printf("Usage : %s <file name containing system configuration data>", argv[0]);
    else
    {
		clock_t start = clock();
		
        threadCount = strtol(argv[2], NULL, 10);
        initiateSystem(argv[1]);
        out_txt = fopen("output.txt", "w");
        out_csv = fopen("output", "w");
        fprintf(out_txt, "Body   :     x              y           vx              vy   \n");
        fprintf(out_csv, "t");
        for (int i = 0; i < bodies; i++)
        {
            fprintf(out_csv, ",x%d,y%d", i + 1, i + 1);
        }
        
        for (int i = 0; i < bodies; i++)
        {
            for (int j = 0; j < bodies; j++)
            {
                sem_init(&semsCompute[i * bodies + j], 0, 0);
            }
        }
        for (int i = 0; i < threadCount + 1; i++) 
        {
            sem_init(&semsPrint[i], 0, 0);
        }
        sem_post(&semsPrint[threadCount]);
        
        for (long i = 0; i < threadCount; i++) 
        {
            pthread_create(&threadHandles[i], NULL, routine, (void*)i);
        }
        
        for (int i = 0; i < timeSteps; i++)
        {
            // главный поток первым печатает заголовок, отмечающий время, и разрешает печать первому потоку
            sem_wait(&semsPrint[threadCount]);
            fprintf(out_txt, "\nCycle %d\n", i + 1);
            fprintf(out_csv, "\n%d", i + 1);
            sem_post(&semsPrint[0]);
        }
        
        for (long i = 0; i < threadCount; i++) 
        {
            pthread_join(threadHandles[i], NULL);
        }
        for (int i = 0; i < threadCount + 1; i++) 
        {
            sem_destroy(&semsPrint[i]);
        }
        for (int i = 0; i < bodies; i++)
        {
            for (int j = 0; j < bodies; j++)
            {
                sem_destroy(&semsCompute[i * bodies + j]);
            }
        }
        fclose(out_txt);
        fclose(out_csv);
        closeSystem();
		
		clock_t end = clock();
		clock_t duration = end - start;
		double duration_sec = (double)duration / (double)CLOCKS_PER_SEC;
		printf("Time: %f seconds\n", duration_sec);
    }
    return 0;
}