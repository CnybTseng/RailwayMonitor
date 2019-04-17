#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int EditConfigFile(const char *filename, char *key, char *value);

void main()
{
	char key[] = "\"verticalCurvature\":";
	char value[128];
	sprintf(value, "%e", 1.35689745e-005);
	EditConfigFile("RailwayMonitor.json", key, value);
}

int EditConfigFile(const char *filename, char *key, char *value)
{
	FILE *fp = NULL;
	FILE *newfp = NULL;
	
	fp = fopen(filename, "r");
	if (!fp) {
		fprintf(stderr, "fopen[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	newfp = fopen("temp.json", "w");
	if (!newfp) {
		fprintf(stderr, "fopen[%s:%d].\n", __FILE__, __LINE__);
		if (fp) fclose(fp);
		return -1;
	}
	
	while (!feof(fp)) {
		char line[1024];
		fgets(line, sizeof(line), fp);
		
		char *p = strstr(line, key);
		if (!p) {
			fputs(line, newfp);
		} else {
			char *q = strpbrk(p, ",\n");
			if (q) {
				char newline[1024];
				size_t len = p - line + strlen(key);
				memcpy(newline, line, len);
				strcpy(newline + len, value);
				strcat(newline, q);
				fputs(newline, newfp);
			} else {
				fprintf(stderr, "End of the line is invalid[%s:%d]!", __FILE__, __LINE__);
				if (fp) fclose(fp);
				if (newfp) fclose(newfp);
				return -1;
			}
		}
	}
	
	if (fp) fclose(fp);
	if (newfp) fclose(newfp);
	
	char cmd[128];
	sprintf(cmd, "move temp.json %s", filename);
	system(cmd);
	
	return 0;
}