//
//  Sudoku.m
//  sudokuSolver
//
//  Created by Gary Liang on 5/25/18.
//  Copyright © 2018 Gary Liang. All rights reserved.
//

#import <Foundation/Foundation.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <set>
#include <vector>
#include <unordered_set>
#include <string>
#include <map>

#import <opencv2/opencv.hpp>
#import <opencv2/videoio/cap_ios.h>
#import <opencv2/imgcodecs/ios.h>
#include "Sudoku.h"

using namespace cv;
using namespace std;

template<int gridSize>
class Sudoku
{
public:
    Sudoku() : grid(gridSize*gridSize, vector<unordered_set<char>>(gridSize*gridSize))
    {
        for(int i = 0; i < gridSize * gridSize; i++)
        {
            restrictions.push_back(ColumnRestriction(i));
            restrictions.push_back(RowRestriction(i));
            restrictions.push_back(BlockRestrictions(i));
        }
    }
    
   // Solve the sudoku board given a fixed number of iterations.
    bool solveSudoku()
    {
        int numOfIterations = 200;
        int currentIteration = 0;
        while(not bruteForceSolve())
        {
            for(auto construct:restrictions)
                construct.scanRecognition(grid);
            currentIteration++;
            if(currentIteration>numOfIterations)
                return false;
        }
        
        return true;
    }
    
    // When this function is called from camera class, it will set values to the 2DVector.
    void setValue(int xPosition, int yPosition, int value)
    {
        if (value==0)
            for(int i = 0; i < gridSize * gridSize; i++)
                grid[xPosition][yPosition].insert(i+1);
        else
            grid[xPosition][yPosition].insert(value);
    }
    
    //getter function
    char getValue(int xPosition, int yPosition)
    {
        if(grid[xPosition][yPosition].size()==1)
            return *grid[xPosition][yPosition].begin();
        else
            return 0;
    }
    
    
    //Brute force algorithm to solve the sudoku puzzle.
    bool bruteForceSolve()
    {
        bool solved = true;
        for(int i = 0; i < gridSize * gridSize; i++)
            for(int j = 0; j < gridSize * gridSize; j++)
                if(grid[i][j].size()!= 1)
                {
                    solved = false;
                    i = gridSize * gridSize;
                    j = gridSize * gridSize;
                }
        return solved;
    }
    
//Private fields and methods. Essential methods to solve sudoku board is here.
private:
    
    struct Restrictions
    {
        //Constructor
        Restrictions(): rowPosition(gridSize * gridSize), columnPosition(gridSize * gridSize)
        {
        }
        
        std::vector<int> rowPosition;
        std::vector<int> columnPosition;
        
        //Scans the positioning of the board, then attempts to solve by brute force.
        bool scanRecognition(vector<vector<unordered_set<char>>>& existingPosition)
        {
            bool isItSolved = false;
            
            unordered_set<char> possibilities;
            for(char i = 0; i < gridSize * gridSize; i++)
                possibilities.insert(i+1);
            
            unordered_set<char> solved;
            for(int i = 0; i < gridSize * gridSize; i++)
                if (existingPosition[rowPosition[i]][columnPosition[i]].size()==1)
                    solved.insert(*(existingPosition[rowPosition[i]][columnPosition[i]].begin()));
            
            for(int i = 0; i < gridSize * gridSize; i++)
                if (existingPosition[rowPosition[i]][columnPosition[i]].size()>1)
                {
                    for(auto track:solved)
                        existingPosition[rowPosition[i]][columnPosition[i]].erase(track);
                    if(existingPosition[rowPosition[i]][columnPosition[i]].size()==1)
                        isItSolved = true;
                    
                }
            
            return isItSolved;
        }
    };
    
    // Row Restrictions
    struct RowRestriction: public Restrictions
    {
        RowRestriction(int row)
        {
            
            for(int i = 0; i < gridSize * gridSize; i++)
            {
                this->rowPosition[i] = row;
                this->columnPosition[i] = i;
            }
        }
        
    };
    
    // Column restrictions
    struct ColumnRestriction: public Restrictions
    {
        ColumnRestriction(int column)
        {
            for(int i = 0; i < gridSize * gridSize; i++)
            {
                this->columnPosition[i] = column;
                this->rowPosition[i] = i;
            }
        }
    };
    
    // block restrictions
    struct BlockRestrictions: public Restrictions
    {
        BlockRestrictions(int block)
        {
            int block_x = block%3;
            int block_y = std::floor(block/3);
            int coordinates = 0;
            for(int i = 0; i < gridSize; i++)
            {
                for(int j = 0; j < gridSize; j++)
                {
                    this->columnPosition[coordinates] = i+block_x* gridSize;
                    this->rowPosition[coordinates] = j+block_y* gridSize;
                    coordinates++;
                }
            }
            
        }
    };
    vector<Restrictions> restrictions;
    
    vector<std::vector<std::unordered_set<char>>> grid;
};

