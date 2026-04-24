#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <functional>

using namespace std;

class Matrix {
private:
    vector<vector<float>> data;
    size_t rows;
    size_t cols;

public:
    // ── Constructeurs ────────────────────────────────────────

    Matrix() : rows(0), cols(0) {}

    Matrix(size_t r, size_t c) : rows(r), cols(c) {
        data.resize(r, vector<float>(c, 0.0f));
    }

    Matrix(initializer_list<initializer_list<float>> init) {
        rows = init.size();
        cols = rows > 0 ? init.begin()->size() : 0;
        data.resize(rows);
        size_t i = 0;
        for (const auto& row : init) {
            if (row.size() != cols)
                throw invalid_argument("Toutes les lignes doivent avoir la meme taille");
            data[i++] = vector<float>(row);
        }
    }

    // ── Accesseurs ───────────────────────────────────────────

    size_t getRows() const { return rows; }
    size_t getCols() const { return cols; }

    float& operator()(size_t i, size_t j) {
        if (i >= rows || j >= cols)
            throw out_of_range("Indices hors limites");
        return data[i][j];
    }

    float operator()(size_t i, size_t j) const {
        if (i >= rows || j >= cols)
            throw out_of_range("Indices hors limites");
        return data[i][j];
    }

    // ── Opérations ───────────────────────────────────────────

    Matrix operator+(const Matrix& other) const {
        if (rows != other.rows || cols != other.cols)
            throw invalid_argument("Les matrices doivent avoir les memes dimensions");
        Matrix result(rows, cols);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                result(i, j) = data[i][j] + other(i, j);
        return result;
    }

    Matrix operator-(const Matrix& other) const {
        if (rows != other.rows || cols != other.cols)
            throw invalid_argument("Les matrices doivent avoir les memes dimensions");
        Matrix result(rows, cols);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                result(i, j) = data[i][j] - other(i, j);
        return result;
    }

    Matrix operator*(const Matrix& other) const {
        if (cols != other.rows)
            throw invalid_argument("Dimensions incompatibles pour la multiplication");
        Matrix result(rows, other.cols);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < other.cols; j++) {
                float sum = 0.0f;
                for (size_t k = 0; k < cols; k++)
                    sum += data[i][k] * other(k, j);
                result(i, j) = sum;
            }
        return result;
    }

    Matrix operator*(float scalar) const {
        Matrix result(rows, cols);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                result(i, j) = data[i][j] * scalar;
        return result;
    }

    Matrix operator/(float scalar) const {
        if (abs(scalar) < 1e-12f)
            throw invalid_argument("Division par zero");
        return (*this) * (1.0f / scalar);
    }

    // ── Méthodes matricielles ────────────────────────────────

    Matrix transpose() const {
        Matrix result(cols, rows);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                result(j, i) = data[i][j];
        return result;
    }

    static Matrix identity(size_t n) {
        Matrix result(n, n);
        for (size_t i = 0; i < n; i++)
            result(i, i) = 1.0f;
        return result;
    }

    // ── Inverse robuste (régularisation de Tikhonov) ─────────────────────────
    //  Calcule (A + λI)⁻¹ avec λ = eps * ||A||_max pour éviter la singularité.
    //  Utilisé à la place de inverse() quand la matrice peut être mal conditionnée.
    Matrix robustInverse(float eps = 1e-6f) const {
        if (rows != cols)
            throw invalid_argument("Seules les matrices carrees peuvent etre inversees");
        // Calcul de ||A||_max
        float maxVal = 0.0f;
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                maxVal = max(maxVal, abs(data[i][j]));
        float lambda = eps * (maxVal > 0.0f ? maxVal : 1.0f);

        // A_reg = A + λI
        Matrix reg = *this;
        for (size_t i = 0; i < rows; i++)
            reg(i, i) += lambda;
        return reg.inverse();
    }

    Matrix inverse() const {
        if (rows != cols)
            throw invalid_argument("Seules les matrices carrees peuvent etre inversees");
        size_t n = rows;
        Matrix aug(n, 2 * n);
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) {
                aug(i, j)     = data[i][j];
                aug(i, j + n) = (i == j) ? 1.0f : 0.0f;
            }
        for (size_t i = 0; i < n; i++) {
            // Pivot partiel pour la stabilité numérique
            size_t maxRow = i;
            float  maxVal = abs(aug(i, i));
            for (size_t k = i + 1; k < n; k++) {
                if (abs(aug(k, i)) > maxVal) {
                    maxVal = abs(aug(k, i));
                    maxRow = k;
                }
            }
            if (maxRow != i)
                for (size_t j = 0; j < 2 * n; j++)
                    swap(aug(i, j), aug(maxRow, j));

            float pivot = aug(i, i);
            if (abs(pivot) < 1e-10f)
                throw runtime_error("Matrice singuliere - non inversible");
            for (size_t j = 0; j < 2 * n; j++)
                aug(i, j) /= pivot;
            for (size_t k = 0; k < n; k++) {
                if (k != i) {
                    float factor = aug(k, i);
                    for (size_t j = 0; j < 2 * n; j++)
                        aug(k, j) -= factor * aug(i, j);
                }
            }
        }
        Matrix result(n, n);
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++)
                result(i, j) = aug(i, j + n);
        return result;
    }

    // ── Jacobienne numérique ─────────────────────────────────
    //
    // Calcule la jacobienne de la fonction vectorielle f : R^n → R^m
    // évaluée au point x, par différences finies centrées.
    //
    //   J(i,j) = ∂f_i / ∂x_j  ≈  [f(x + ε·eⱼ) - f(x - ε·eⱼ)] / (2ε)
    //
    // f    : fonction (Matrix colonne → Matrix colonne)
    // x    : point d'évaluation (n×1)
    // outDim : dimension de sortie m  (nécessaire pour allouer J)
    // eps  : pas de différentiation (défaut 1e-4)
    //
    static Matrix jacobian(
        const function<Matrix(const Matrix&)>& f,
        const Matrix& x,
        size_t outDim,
        float eps = 1e-4f)
    {
        size_t n = x.getRows();
        Matrix J(outDim, n);

        for (size_t j = 0; j < n; j++) {
            Matrix xp = x;  xp(j, 0) += eps;
            Matrix xm = x;  xm(j, 0) -= eps;
            Matrix fp = f(xp);
            Matrix fm = f(xm);
            for (size_t i = 0; i < outDim; i++)
                J(i, j) = (fp(i, 0) - fm(i, 0)) / (2.0f * eps);
        }
        return J;
    }

    // ── Affichage ────────────────────────────────────────────

    void print(const string& name = "") const {
        if (!name.empty())
            cout << name << " =\n";
        for (size_t i = 0; i < rows; i++) {
            cout << "[ ";
            for (size_t j = 0; j < cols; j++) {
                cout << setw(10) << fixed << setprecision(4) << data[i][j];
                if (j < cols - 1) cout << ", ";
            }
            cout << " ]\n";
        }
        cout << endl;
    }
};

#endif // MATRIX_HPP
