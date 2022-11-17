#include "Header.h"

int main()
{
	memorisation();
	Sleep(5000);
	lire_fichier();

}

void memorisation(void)
{
	char date[9], heure_debut[9], heure_fin[9];
	char caractere;
	errno_t erreur;
	FILE * mon_fichier = NULL;
	erreur = fopen_s(&mon_fichier, "espion.txt", "w");

	if (erreur == 0)
	{
		_strdate_s(date, 9);
		_strtime_s(heure_debut);

		fputs(date, mon_fichier);
		fputc('\n', mon_fichier);
		fputs(heure_debut, mon_fichier);
		fputc('\n', mon_fichier);

		
		while ((caractere = _getch()) != 27 )
		{
			fputc(caractere, mon_fichier);

			if (caractere == 13)
			{
				caractere = '\n';
			}
			putc(caractere, stdout); // ecrit dans la console
		}
		system("cls");
		fputc('\n', mon_fichier);
		fputc(27, mon_fichier);
		fputc('\n', mon_fichier);
		_strtime_s(heure_fin);
		fputs(heure_fin, mon_fichier);
		fclose(mon_fichier);
	}
	else
	{
		printf("MARCHE PAS\n");
	}

}


void lire_fichier(void)
{
	char chaine[10];
	char caractere;
	errno_t erreur;
	FILE * mon_fichier = NULL;

	erreur = fopen_s(&mon_fichier, "espion.txt", "r");



	if (erreur == 0)
	{
		
		fgets(chaine, 10, mon_fichier);
		fgets(chaine, 10, mon_fichier);

		
		while ((caractere= fgetc(mon_fichier)) != 27)
		{
			if (caractere == 13)
			{
				caractere = '\n';
			}
			putc(caractere, stdout);
		}
		fclose(mon_fichier);
	}
	else
	{
		printf("MARCHE PAS\n");

	}
}
