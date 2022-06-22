#include <iostream>
#include <vector>
#include <cmath>


struct move {
    int r, c, score;
};
std::vector<std::vector<int>> empty_cell(std::vector<std::vector<char>> grid){
    std::vector<std::vector<int>> tmp;
    
    for (int i=0;i<grid.size();i++){
        for ( int j =0;j<grid.size();j++){
            if ( grid[i][j] == ' '){
                std::vector<int> loc = {i,j};
                tmp.push_back(loc);
            }
        }
    }
    
    return tmp;
} 
std::vector<std::vector<char>> Create_grid(int size) {
	std::vector<std::vector<char>> grid;
	for (int i = 0; i < size; i++) {
		grid.push_back(std::vector<char>());
		for (int j = 0; j < size; j++) {
			grid[i].push_back(' ');
		}
	}
	return grid;
}

int check_win(std::vector<char> arr, int wincond){
    int counterO = 0;
    int counterX =  0;
	for (int j = 0; j < arr.size(); j++) {
		if (arr[j] == 'O')
		{
			if (counterX != 0)
			{
				counterX = 0;
			}					
			counterO++;
		}
		if (arr[j] == 'X')
		{
			if (counterO != 0)
			{
				counterO = 0;
			}
			
			counterX++;
		}


		if (counterO == wincond)
		{
			return 1;	
		}
		else if(counterX == wincond)
		{

			return -1;
			
		}
	
	}
	return 0;
}

char win(std::vector<std::vector<char>> grid, int wincond ) {
    
    int s = grid.size();
	int win_O = 0;
	int win_X = 0;
	int draw = 0;
    
    for ( int i=0;i<s-wincond+1;i++){
        std::vector<char> tmp[4];  
        for ( int j = 0; j< s-i; j++){
            tmp[0].push_back(grid[i+j][j]);
            tmp[1].push_back(grid[j][i+j]);
            tmp[2].push_back(grid[s-i-1-j][j]);
            tmp[3].push_back(grid[s-j-1][i+j]);
        }
        for (auto elm:tmp){
 
            if ( check_win(elm,wincond) == 1){
                win_O++;
            }
            else if ( check_win(elm,wincond) == -1){
                win_X++;
            }
        }
				
    }
    
    for ( int i=0;i<s;i++){
        
        std::vector<char> loc;
        for ( int j=0;j<s;j++){
            loc.push_back(grid[j][i]);
        }
        
        
        if ( check_win(grid[i],wincond) == 1 || check_win(loc,wincond) == 1 ){
            win_O++;
        }
        else if (  check_win(grid[i],wincond) == -1 || check_win(loc,wincond) == -1 ){
            win_X++;
        }
    }
 
        
	if (win_X > 0){
	    return 'X';
	}			
	else if(win_O > 0){
	    return 'O';
	}
	
	for ( auto row:grid){
	    for (auto elm:row){
	        if (elm == ' ') { draw++;}
	    }
	}
	
	if ( draw == 0) {
	    return 'd';
	}
	 
	return 'n';
	
	
	
// 	else {
	    
        
// 	    return 0;
// 	}
    
}

void print(std::vector<std::vector<char>> grid) {
	std::cout << '\n';
	for (int i = 0; i < grid.size(); i++) {
		if (i) {
			for (int i = 0; i < grid.size(); i++)
			{
				std::cout << "-----";
			}
			std::cout << "\n";	
		}
		for (int j = 0; j < grid.size(); j++) {
			if (j) {
				std::cout << "|";
			}
			std::cout << ' ';
			if (grid[i][j] == ' ') {
                    std::cout << grid.size() * i + j + 1;
                } else {
                    std::cout << grid[i][j];
                }
			std::cout << ' ';
		}
		std::cout << '\n';
	}
	std::cout << '\n';
}

int tie(std::vector<std::vector<char>> grid,int wincond){
	if(win(grid,wincond)){
		return -1;
	}
	for (int i = 0; i < grid.size(); i++)
	{
		for (int j = 0; j < grid.size(); j++)
		{
			if (grid[i][j] != ' ')
			{
				return -1;
			}
			
		}	
	}
	return 0;	
}

std::vector<std::vector<char>> player_move(std::vector<std::vector<char>> grid,char p_marker) {
	std::cout << "Enter the empty cell:"; 
	int cell;
	std::cin >> cell;
	int last_cell = pow(grid.size(),2);
	int i = (cell -1)/grid.size(), j = (cell-1)%grid.size();
	if(cell >= 1 && cell <= last_cell) {
		grid[i][j] = p_marker;
	}
	else{
		std:: cout << "\n Please enter the correct cell !!! \n";
	}

	return grid;
}

int minimax(std::vector<std::vector<char>> grid,int wincond,char marker,int deepth, bool maximizing_player){
    
    if ( win(grid,wincond) == 'd') {
        return 0;
    }
    
     char opposite = marker =='X' ? 'O' : 'X';

    deepth ++;
    if ( maximizing_player){
        
        if ( win(grid,wincond) == marker) {
            return -1;
        }
        if ( win(grid,wincond) == opposite) {
            return 1;
        }
     
        int maxEval = -999;
        
        std::vector<std::vector<int>> free_move = empty_cell(grid);
       
        int eval;
        for( auto elm:free_move){
            grid[elm[0]][elm[1]] = opposite;
            eval = minimax(grid,wincond, opposite,deepth , false);
            if(eval > maxEval ){
                maxEval = eval;
 
            }
            grid[elm[0]][elm[1]] = ' ';
        }
        
        return eval;
    }
    else {
          if ( win(grid,wincond) == opposite) {
                return -1;
            }  
        if ( win(grid,wincond) == marker) {
                return 1;
            }      
            
        int minEval = 999;
        
        std::vector<std::vector<int>> free_move = empty_cell(grid);
     
        int eval;
        for( auto elm:free_move){
            grid[elm[0]][elm[1]] = opposite;
            eval = minimax(grid,wincond, opposite,deepth , true);
            if(eval < minEval ){
                minEval = eval;

            }
            grid[elm[0]][elm[1]] = ' ';
        }

        return eval;
        
        
    }
 


}

std::vector<std::vector<char>> computer_move(std::vector<std::vector<char>> grid, char marker, int wincond) {
    
    std::vector<std::vector<int>> free_move = empty_cell(grid);
    
    if(free_move.size() > 0)
    {
        int val = -999;
    std::vector<int> move;
    
    for (auto elm:free_move){
        grid[elm[0]][elm[1]] = marker;
        int tmp = minimax(grid,wincond,marker,1, false) ;
        if (  tmp > val){
            val = tmp;
            move = elm;
        }       
        grid[elm[0]][elm[1]] = ' ';
    }
    
// 	std::cout << move[0] << std::endl;
// 	std::cout << move[1] << std::endl;
// 	std::cout << val << std::endl;
    
    grid[move[0]][move[1]] = marker;
	
    }
    return grid;
}

void play(std::vector<std::vector<char>> grid, int wincond){
	char player,computer;
	while (true){
		std::cout <<" Which sybol go first (X or O)?";
		std::cin >> player;
		if(player == 'X' || player == 'O'){
			break;
		}
		std::cout <<" Please input as X or O!";
	}
	
	
	
	computer = player =='X' ? 'O' : 'X';
// 	if (player == 'O')
// 	{	
// 		std:: cout <<1;
// 		grid = computer_move(grid,computer,wincond);
// 		std:: cout <<2;
// 	}
	
	
	print(grid);
	while(1){
	    if (win(grid,wincond) == 1) {
	        std::cout << "Player O win" << std::endl;
	        break;
	    }
	    if (win(grid,wincond) == -1) {
	        std::cout << "Player X win" << std::endl;
	        break;
	    }
	    	    if (win(grid,wincond) == 0) {
	        std::cout << "Draw!" << std::endl;
	        break;
	    }
	    
	    
	    grid = player_move(grid,player);
		print(grid);
	    grid = computer_move(grid,computer,wincond);
	    

		print(grid);
	}
}




int main()
{
    // std::vector<std::vector<char>> grid;
    char player = 'X';
    
    std::vector<std::vector<char>> grid ;
                                           
    // std::vector<char> tmp = {'O','O','O'};        
    //         std::cout<<check_win(tmp,3)<<std::endl;
    
    // std::cout<<win(grid,3) ;                            
    // std::vector<int> elm = {0,2};
    // std::cout<<minimax(grid,3,'X',1,false);
    
	grid = Create_grid(3);
	
// 	std::vector<std::vector<int>> a = empty_cell(grid);
    
    // for (auto elm :a){
    //     for( auto x : elm){
    //         std::cout<<x<<std::endl;
    //     }
    // }
// 	print(a);
	play(grid,3);
    return 0;
}