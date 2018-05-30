// CSmoothing.hpp

#ifndef __CSMOOTHING_HPP
#define __CSMOOTHING_HPP

// D E F I N E S ---------------------------------------------------------------------
#define MAX_ARRAY_SIZE 4                            // Size of buffered elems, higher values results in better smoothing,
                                                    // lower in better identification of different values
// C L A S S - C S M O O T H I N G --------------------------------------------------
template <class T> class CSmoothing {
public:
    CSmoothing() {}

    // Add a value to the array, LIFO-principle
    void addValue(T val) {
      array[actElemt] = val;

      if (actElems < MAX_ARRAY_SIZE)
        actElems++;

      if (actElemt+1 == MAX_ARRAY_SIZE)
        actElemt = 0;
      else
        actElemt++;
    } // ADDVALUE

    // Get the avaerage of the buffered elements
    T getAverage() {
      if (actElems == 0)
        return 0;

        T sumOfVals = 0;

        for (int i=0; i<actElems; i++)
          sumOfVals += array[i];

        return (T)(sumOfVals / actElems);
    } // GETAVERAGE

private:
    T array[MAX_ARRAY_SIZE];                        // The Array with the buffered values
    int actElemt = 0;                               // Actual element
    int actElems = 0;                               // Actual number of elements
}; // CLASS CSMOOTHING

#endif // __CSMOOTHING_HPP
