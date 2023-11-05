#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <pthread.h>
#include <semaphore.h>

#define DT 0.05

typedef struct
{
    double x, y;
} vector;

int bodies, timeSteps, threadCount, localBodies;
double *masses, GravConstant;
vector *positions, *velocities, *accelerations;
sem_t *semsPrint;
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

void computeAccelerations(int first, int last)
{
    int i, j;
    float denom;

    for (i = first; i < last; i++)
    {
        accelerations[i].x = 0;
        accelerations[i].y = 0;
        for (j = 0; j < bodies; j++)
        {
            if (i != j)
            {
                denom = pow(mod(subtractVectors(positions[i], positions[j])), 3);
                if (denom < FLT_EPSILON) denom = FLT_EPSILON; // вместо ResolveCollisions
                accelerations[i] = addVectors(accelerations[i], scaleVector(GravConstant * masses[j] / denom, subtractVectors(positions[j], positions[i])));
            }
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
    if(myLastBody > bodies) myLastBody = myFirstBody; // на случай, если потоков больше, чем тел
    if(myRank == threadCount - 1) myLastBody = bodies; // чтобы последний поток захватил остатки
    
    for (int i = 0; i < timeSteps; i++)
    {
        // считаем формулки
        computeAccelerations(myFirstBody, myLastBody);
        computePositions(myFirstBody, myLastBody);
        computeVelocities(myFirstBody, myLastBody);
        
        //печатаем числа
        sem_wait(&semsPrint[myRank]);
        printBody(myFirstBody, myLastBody);
        sem_post(&semsPrint[myRank + 1]);
    }
    
    return NULL;
}

int main(int argc, char *argv[])
{
    int i, j;

    if (argc != 3) // используем три агрумента: файл ввода, файл вывода, число потоков
        printf("Usage : %s <file name containing system configuration data>", argv[0]);
    else
    {
        threadCount = strtol(argv[2], NULL, 10);
        pthread_t* threadHandles = malloc(threadCount * sizeof(pthread_t));
        semsPrint = malloc((threadCount + 1) * sizeof(sem_t)); // [threadCount] будет использоваться для главного потока
        
        initiateSystem(argv[1]);
        out_txt = fopen("output.txt", "w");
        out_csv = fopen("output", "w");
        fprintf(out_txt, "Body   :     x              y           vx              vy   \n");
        fprintf(out_csv, "t");
        for (int i = 0; i < bodies; i++)
        {
            fprintf(out_csv, ",x%d,y%d", i + 1, i + 1);
        }
        
        sem_post(&semsPrint[threadCount]);
        
        for(long i = 0; i < threadCount; i++) 
        {
            sem_init(&semsPrint[i], 0, 0);
        }
        for(long i = 0; i < threadCount; i++) 
        {
            pthread_create(&threadHandles[i], NULL, routine, (void*)i);
        }
        
        for (i = 0; i < timeSteps; i++)
        {
            sem_wait(&semsPrint[threadCount]);
            fprintf(out_txt, "\nCycle %d\n", i + 1);
            fprintf(out_csv, "\n%d", i + 1);
            sem_post(&semsPrint[0]);
        }
        
        for(long i = 0; i < threadCount; i++) 
        {
            pthread_join(threadHandles[i], NULL);
        }
        for(long i = 0; i < threadCount; i++) 
        {
            sem_destroy(&semsPrint[i]);
        }
        fclose(out_txt);
        fclose(out_csv);
        free(threadHandles);
        free(semsPrint);
    }
    return 0;
}