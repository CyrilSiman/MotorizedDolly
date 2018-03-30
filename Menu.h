#ifndef _MENU_H_
#define _MENU_H_
 
/* Structure d'un menu */
/*
typedef struct {
  const char** items;     // Tableau de choix du menu
  const uint8_t nbItems;  // Nombre de choix possibles
  const uint8_t level; //0 for main menu, 1 for sub menu, 2 for sub-sub menu ...
  //void (*callbackFnct)(uint8_t menuItemSelected); // Pointeur sur fonction pour gérer le choix de l'utilisateur
} Menu_t;
*/

enum MenuType {MENU_TYPE_MENU, MENU_TYPE_ITEM};

//typedef struct MenuItem MenuItem;

typedef struct MenuItem
{
  MenuType menutype;
  MenuItem *menuItems;
  int menuItemsLenght;
  char* menuPrompt;
  void *callbackFnct;
};

typedef struct MenuLevel
{
  MenuItem menu;
  int currentMenuPos = 1;
  int firstItemDisplayed = 1;
};

#endif
