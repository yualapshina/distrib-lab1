#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define DT 0.05

typedef struct
{
    double x, y;
} vector;

int bodies, timeSteps;
double *masses, GravConstant;
vector *positions, *velocities, *accelerations;

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

void resolveCollisions()
{
    int i, j;

    for (i = 0; i < bodies - 1; i++)
        for (j = i + 1; j < bodies; j++)
        {
            if (positions[i].x == positions[j].x && positions[i].y == positions[j].y)
            {
                vector temp = velocities[i];
                velocities[i] = velocities[j];
                velocities[j] = temp;
            }
        }
}

void computeAccelerations()
{
    int i, j;

    for (i = 0; i < bodies; i++)
    {
        accelerations[i].x = 0;
        accelerations[i].y = 0;
        for (j = 0; j < bodies; j++)
        {
            if (i != j)
            {
                accelerations[i] = addVectors(accelerations[i], scaleVector(GravConstant * masses[j] / pow(mod(subtractVectors(positions[i], positions[j])), 3), subtractVectors(positions[j], positions[i])));
            }
        }
    }
}

void computeVelocities()
{
    int i;

    for (i = 0; i < bodies; i++)
        velocities[i] = addVectors(velocities[i], scaleVector(DT, accelerations[i]));
}

void computePositions()
{
    int i;

    for (i = 0; i < bodies; i++)
        positions[i] = addVectors(positions[i], scaleVector(DT,velocities[i]));
}

void simulate()
{
    computeAccelerations();
    computePositions();
    computeVelocities();
    resolveCollisions();
}

int main(int argc, char *argv[])
{
    int i, j;

    if (argc != 3)
        printf("Usage : %s <file name containing system configuration data>", argv[0]);
    else
    {
        initiateSystem(argv[1]);
        FILE *out_txt = fopen("output.txt", "w");
        FILE *out_csv = fopen("output", "w");
        fprintf(out_txt, "Body   :     x              y           vx              vy   \n");
        fprintf(out_csv, "t");
        for (int i = 0; i < bodies; i++)
        {
            fprintf(out_csv, ",x%d,y%d", i + 1, i + 1);
        }
        for (i = 0; i < timeSteps; i++)
        {
            fprintf(out_txt, "\nCycle %d\n", i + 1);
            fprintf(out_csv, "\n%d", i + 1);
            simulate();
            for (j = 0; j < bodies; j++)
            {
                fprintf(out_txt, "Body %d : %lf\t%lf\t%lf\t%lf\n", j + 1, positions[j].x, positions[j].y, velocities[j].x, velocities[j].y);
                fprintf(out_csv, ",%lf,%lf", positions[j].x, positions[j].y);
            }
        }
        fclose(out_txt);
        fclose(out_csv);
    }
    return 0;
}