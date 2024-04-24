#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "MathMatrix.hpp"

/**
 * @brief 
 * 
 * @tparam S 
 */
template <size_t S>
class KalmanFilter
{
protected:
    double h[1][S];
    double ht[S][1];
    double r;
    double phi[S][S];
    double phit[S][S];
    double gamma[S][S];
    double q[S][S];
    double I[S][S];
public:
    /**
     * @brief Construct a new Kalman Filter object
     * 
     */
    KalmanFilter();
    /**
     * @brief 
     * 
     * @param xk 
     * @param pk 
     * @param y 
     * @param uk 
     * @param xk1 
     * @param pk1 
     */
    void predict(double xk[S][1], double pk[S][S], double y[S][1], double uk[S][1], double xk1[S][1], double pk1[S][S]);
    /**
     * @brief Set the params object
     * 
     * @param h 
     * @param r 
     * @param phi 
     * @param gamma 
     * @param q 
     * @param x0
     */
    void set_params(double h[1][S], double r, double phi[S][S], double gamma[S][S], double q[S][S]);
    void show_status()
    {
        Serial.print("{h: ");
        Serial.print(h[0][0]);
        Serial.print(", ht: ");
        Serial.print(ht[0][0]);
        Serial.print(", r: ");
        Serial.print(r);
        Serial.print(", phi: ");
        Serial.print(phi[0][0]);
        Serial.print(", phit: ");
        Serial.print(phit[0][0]);
        Serial.print(", gamma: ");
        Serial.print(gamma[0][0]);
        Serial.print(", q: ");
        Serial.print(q[0][0]);
        Serial.print(", I: ");
        Serial.print(I[0][0]);
        Serial.println("}");
    }
};

/**
 * @brief Construct a new Kalman Filter< S>:: Kalman Filter object
 * 
 * @tparam S 
 */
template <size_t S>
KalmanFilter<S>::KalmanFilter()
{
    for (size_t i = 0; i < S; ++i)
        for (size_t j = 0; j < S; ++j)
        {
            if (i == j)
                this->I[i][j] = 1.0;
            else
                this->I[i][j] = 0.0;
        }
}

/**
 * @brief 
 * 
 * @tparam S 
 * @param xk 
 * @param pk 
 * @param y 
 * @param uk 
 * @param xk1 
 * @param pk1 
 */
template <size_t S>
void KalmanFilter<S>::predict(double xk[S][1], double pk[S][S], double y[S][1], double uk[S][1], double xk1[S][1], double pk1[S][S])
{
    // Gain
    // Pk * H' -> [S][S] * [S][1] = [S][1]
    double pk_ht[S][1];
    matmult<double, S, S, 1>(pk, this->ht, pk_ht);

    // H * Pk -> [1][S] * [S][S] = [1][S]
    // H * (Pk * H') -> [1][S] * [S][1] = [1][1]
    double h_pk_ht[1][1];
    matmult<double, 1, S, 1>(this->h, pk_ht, h_pk_ht);
    // H * Pk * H' + R -> [1][1]
    add<double, 1, 1>(h_pk_ht, this->r);
    // inv(H * Pk * H' + R)
    double h_pk_ht_inv[1][1] = {1 / h_pk_ht[0][0]};
    // (Pk * H') * inv(H * Pk * H' + R) -> [S][1] * [1][1] -> [S][1]
    double k[S][1];
    scale<double, S, 1>(pk_ht, h_pk_ht_inv[0][0], k);

    // Update
    // -- x(k+1)
    // ---- H * xk -> [1][S] * [S][1] = [1][1]
    double h_xk[1][1];
    matmult<double, 1, S, 1>(this->h, xk, h_xk);
    // ---- y - (H * xk) -> [S][1] * [1][1] = [S][1]
    double y_h_xk[S][1];
    sub<double, S, 1>(y, h_xk[0][0], y_h_xk);
    // ---- K * (y - H * xk) -> [S][1] * [S][1] = [S][1]
    double k_y_h_xk[S][1];
    vecmult<double, S>(y_h_xk, k, k_y_h_xk);
    // ---- xkplus = xk + K * (y - H * xk) -> [S][1] + [S][1] = [S][1]
    double xkplus[S][1];
    matadd<double, S, 1>(k_y_h_xk, xk, xkplus);
    // -- P(k+1)
    // ---- K * H -> [S][1] * [1][S] = [S][S]
    double k_h[S][S];
    matmult<double, S, 1, S>(k, this->h, k_h);
    // ---- I - (K * H) -> [S][S] - [S][S] = [S][S]
    double i_k_h[S][S];
    sub<double, S, S>(this->I, k_h, i_k_h);
    // ---- Pkplus = (I - K * H) * Pk -> [S][S] * [S][S] = [S][S]
    double pkplus[S][S];
    matmult<double, S, S, S>(i_k_h, pk, pkplus);

    // Propagation
    // -- x(k + 1)
    // ---- Gamma * uk -> [S][S] * [S][1] = [S][1]
    double gamma_uk[S][1];
    matmult<double, S, S, 1>(this->gamma, uk, gamma_uk);
    // ---- Phi * xkplus -> [S][S] * [S][1] = [S][1]
    double phi_xkplus[S][1];
    matmult<double, S, S, 1>(this->phi, xkplus, phi_xkplus);
    // ---- Gamma * uk + Phi * xkplus -> [S][S] * [S][1] = [S][1]
    matadd<double, S, 1>(gamma_uk, phi_xkplus, xk1);
    // -- P(k + 1)
    // ---- Phi * Pkplus -> [S][S] * [S][S] = [S][S]
    double phi_pkplus[S][S];
    matmult<double, S, S, S>(this->phi, pkplus, phi_pkplus);
    // ---- (Phi * Pkplus) * Phi^t -> [S][S] * [S][S] = [S][S]
    double phi_pkplus_phit[S][S];
    matmult<double, S, S, S>(phi_pkplus, this->phit, phi_pkplus_phit);
    // ---- (Phi * Pkplus) * Phi^t + Q -> [S][S] * [S][S] = [S][S]
    matadd<double, S, S>(phi_pkplus_phit, this->q, pk1);
}

/**
 * @brief 
 * 
 * @tparam S 
 * @param h 
 * @param r 
 * @param phi 
 * @param gamma 
 * @param q 
 */
template <size_t S>
void KalmanFilter<S>::set_params(double h[1][S], double r, double phi[S][S], double gamma[S][S], double q[S][S])
{
    this->r = r;

    for (size_t i = 0; i < S; ++i)
    {
        this->h[0][i] = h[0][i];

        for (size_t j = 0; j < S; ++j)
        {
            this->phi[i][j] = phi[i][j];
            this->gamma[i][j] = gamma[i][j];
            this->q[i][j] = q[i][j];
        }
    }

    transpose<double, 1, S>(this->h, this->ht);
    transpose<double, S, S>(this->phi, this->phit);
}

#endif // KALMAN_FILTER_HPP
