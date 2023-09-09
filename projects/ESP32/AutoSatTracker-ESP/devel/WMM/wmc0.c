/* given wmm.cof on stdin, produce the source code for epoc, c0 and cd0 on stdout for AutoSatTracker
 */

#include <stdio.h>

double c0[13][13], cd0[13][13];

int main (int ac, char *av[])
{
        int n, m;
        double gnm, hnm, dgnm, dhnm, year;

        if (scanf("%lf%*s%*s", &year) == 1)
            printf ("static float epoc = %.1f;\n\n", year);

        while (scanf("%d%d%lf%lf%lf%lf", &n, &m, &gnm, &hnm, &dgnm, &dhnm) == 6) {
            c0[m][n] = gnm;
            cd0[m][n] = dgnm;
            if (m != 0) {
                c0[n][m-1] = hnm;
                cd0[n][m-1] = dhnm;
            }
        }

        printf ("static const float c0[13][13] = {\n");
        for (n = 0; n < 13; n++) {
            printf ("    {");
            for (m = 0; m < 13; m++)
                printf ("%9.1f,", c0[n][m]);
            printf ("},\n");
        }
        printf ("};\n\n");

        printf ("static const float cd0[13][13] = {\n");
        for (n = 0; n < 13; n++) {
            printf ("    {");
            for (m = 0; m < 13; m++)
                printf ("%9.1f,", cd0[n][m]);
            printf ("},\n");
        }
        printf ("};\n\n");

        return (0);
}
