#ifndef MATH_MATRIX_HPP
#define MATH_MATRIX_HPP

template <typename T, size_t M, size_t N>
void matadd(T A[M][N], T B[M][N], T C[M][N])
{
    int i, j;

    for (i = 0; i < M; ++i)
        for(j = 0; j < N; ++j)
            C[i][j] = A[i][j] + B[i][j];
}

template <typename T, size_t M, size_t N>
void add(T A[M][N], T b)
{
    int i, j;

    for (i = 0; i < M; ++i)
        for(j = 0; j < N; ++j)
            A[i][j] = A[i][j] + b;
}

template <typename T, size_t N>
int inv(T A[N][N], T C[N][N])
{
    int pivrow = 0;
    int k, i, j;
    int pivrows[N];
    T tmp[N][N];

    for (k = 0; k < N; k++)
    {
        tmp = 0;

        for (i = k; i < N; ++i)
        {
            if (abs(A[i][k]) >= tmp)
            {
                tmp = abs(A[i][k]);
                pivrow = i;
            }
        }

        if (A[pivrow][k] == 0.0f)
        {
            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }

        if (pivrow != k)
        {
            for (j = 0; j < N; ++j)
            {
                tmp = A[k][j];
                A[k][j] = A[pivrow][j];
                A[pivrow][j] = tmp;
            }
        }

        pivrows[k] = pivrow;

        tmp = 1.0f / A[k][k];
        A[k][k] = 1.0f;

        for (j = 0; j < N; ++j)
        {
            A[k][j] = A[k][j] * tmp;
        }

        for (i = 0; i < N; ++i)
        {
            if (i != k)
            {
                tmp = A[i][k];
                A[i][k] = 0.0f;

                for (j = 0; j < N; ++j)
                {
                    A[i][j] = A[i][j] - A[k][j] * tmp;
                }
            }
        }
    }

    for (k = N - 1; k >= 0; k--)
    {
        if (pivrows[k] != k)
        {
            for (i = 0; i < N; ++i)
            {
                tmp = A[i][k];
                C[i][k] = A[i][pivrows[k]];
                C[i][pivrows[k]] = tmp;
            }
        }
    }

    return 1;
}

template <typename T, size_t M, size_t N, size_t P>
void matmult(T A[M][N], T B[N][P], T C[M][P])
{
    int i, j, k;

    for (i = 0; i < M; ++i)
        for(j = 0; j < N; ++j)
        {
            C[i][j] = 0;

            for (k = 0; k < P; k++)
                C[i][j] = C[i][j] + A[i][k] * B[k][j];
        }
}

template <typename T, size_t M, size_t N>
void scale(T A[M][N], T a)
{
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < N; ++j)
            A[i][j] = a * A[i][j];
}

template <typename T, size_t M, size_t N>
void scale(T A[M][N], T a, T C[M][N])
{
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < N; ++j)
            C[i][j] = a * A[i][j];
}

template <typename T, size_t M, size_t N>
void sub(T A[M][N], T B[M][N], T C[M][N])
{
    int i, j;

    for (i = 0; i < M; ++i)
        for(j = 0; j < N; ++j)
            C[i][j] = A[i][j] - B[i][j];
}

template <typename T, size_t M, size_t N>
void sub(T A[M][N], T b, T C[M][N])
{
    int i, j;

    for (i = 0; i < M; ++i)
        for(j = 0; j < N; ++j)
            C[i][j] = A[i][j] - b;
}

template <typename T, size_t M, size_t N>
void transpose(T A[M][N], T C[N][M])
{
    int i, j;

    for (i = 0; i < M; ++i)
        for(j = 0; j < N; ++j)
            C[j][i] = A[i][j];
}

template <typename T, size_t N>
void vecmult(T A[N][1], T B[N][1], T C[N][1])
{
    for (size_t i = 0; i < N; ++i)
        C[i][0] = A[i][0] * B[i][0];
}

template <typename T, size_t N>
void vecmult(T A[1][N], T B[1][N], T C[1][N])
{
    for (size_t i = 0; i < N; ++i)
        C[0][i] = A[0][i] * B[i][0];
}

#endif // MATH_MATRIX_HPP
