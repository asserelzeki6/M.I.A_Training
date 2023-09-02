// Header files
#include <iostream> // for I/O functions
#include <bits/stdc++.h> // for stls
#include <conio.h> // for getch()

using namespace std;

/// Defining functions used:

// adding a task and viewing the table of tasks
void addTask();

// for marking a task completed by choosing it's id from the shown table
// @param : id of the completed task
void markCompleted(int id);

// basically the function that shows the table of tasks *used in many other functions*
void viewTasks();

// for removing a task by choosing it's id from the table and then viewing the table after the removal
// *parameter : id of the removed task
void removeTask(int id);

//the main function that controls the  flow of the program
void menu ();

/// The container used:
// dimensions are : ID , Task , completed or not
multimap <int,pair<string,bool>> m;


int main()
{
    cout << "Welcome to Minions Task Manager\n\nPress enter for Main Menu";
    getch();
    system("cls");
    menu();
    cout << "Exiting Minions Task Manager. Have a great day!" << endl;
    m.clear();
    return 0;
}

/// Menu function to display the menu and go to the suitable function
void menu()
{
    char choice;
    bool leave=true;
    while(leave)
    {
        cout << "Main menu" <<endl<<endl;
        cout << "1. Add Task" <<endl;
        cout << "2. Mark as completed" <<endl;
        cout << "3. View Tasks" <<endl;
        cout << "4. Remove Task" <<endl;
        cout << "5. Exit" <<endl;
        cout << endl;
        cout << "Select an option: ";
        cin >> choice;
        system("cls");
        int id;
        switch(choice)
        {
        case '1':
            addTask();
            cout << "Task added successfully! " << endl<<endl;
            cout << "Press enter for Main Menu" << endl;
            getch();
            system("cls");
            break;
        case '2':
            viewTasks();
            if(m.size()!=0)
            {
                cout << "ID of the completed task: ";
                cin >> id;
                markCompleted(id);
            }
            cout << "Press enter for Main Menu" << endl;
            getch();
            system("cls");
            break;
        case '3':
            viewTasks();
            cout << "Press enter for Main Menu" << endl;
            getch();
            system("cls");
            break;
        case '4':
            viewTasks();
            if(m.size()!=0)
            {
                cout << "ID of the task you want to remove: ";
                cin >> id;
                removeTask(id);
            }
            cout << "Press enter for Main Menu" << endl;
            getch();
            system("cls");
            break;
        case '5':
            leave=false;

            system("cls");

            break;
        default:
            cout << "oops! \nYou've entered a wrong choice\nPlease choose wisely from the main menu :(\n\n";

        }
    }

}

/// Adding a new task
void addTask()
{
    string task;

    int id=1;
    for(auto it : m)
    {
        id=it.first+1;
    }
    cout << "Enter task description: ";
    getchar();  // for catching the \n
    getline(cin, task);  // reading the whole sentence

    // inserting in the map
    m.insert({id,{task,false}});

    system("cls");
    viewTasks();
}

/// marking a given task completed
void markCompleted(int id)
{
    // If entered a wrong ID
    if(id>m.size() || id<1)
    {
        cout << "There is no task with such ID !" << endl;
        return;
    }

    auto it=m.begin();
    for(int i=1; it!=m.end(); it++,i++)
    {
        if(i==id)
        {
            break;
        }
    }

    // checking whether the given task is already completed or not
    if(it->second.second)
    {
        cout << "Task already completed!" << endl;
        return;
    }
    it->second.second=true;

    // viewing task after finishing
    system("cls");
    viewTasks();
    cout << "Task completed successfully :)" << endl;

}

/// viewing all the tasks given
void viewTasks()
{
    // no tasks to view
    if(m.empty())
    {
        cout << "You have no tasks!" << endl << endl;
        return;
    }

    // printing the table frames
    cout << "Current Tasks\n\n";
    cout << left << setw(20) << "ID" << " | "
         << setw(20) << "Completed" << " | "
         << setw(20) << "Description" << "\n";
    cout << setfill('-') << setw(65) << "" << "\n";
    cout << setfill(' ');

    //printing the table data from the map
    int i=1;
    for(auto it: m)
    {
        cout << left << setw(20) << i << " | ";
        if(it.second.second)
            cout << setw(20) << "YES" << " | ";
        else
            cout << setw(20) << "NO" << " | ";

        cout << setw(20) << it.second.first << "\n";
        i++;
    }
    cout << endl;
}

/// Removing a given task
void removeTask(int id)
{
    // If given a wrong id
    if(id>m.size() || id<1)
    {
        cout << "There is no task with such ID !" << endl<<endl;
        return;
    }

    // traversing to reach the given ID
    auto it=m.begin();
    for(int i=1; it!=m.end(); it++,i++)
    {
        if(i==id)
        {
            break;
        }
    }
    m.erase(it);
    system("cls");
    viewTasks();
    cout << "Task erased successfully :)" <<endl<<endl;
}
