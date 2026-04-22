void identity(int n, double* identity_result) {
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            identity_result[j * n + i] = (i == j) ? 1.0 : 0.0;
        }
    }
}
